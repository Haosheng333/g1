// pcd_to_map - offline converter: FAST-LIO2 PCD map -> ROS map_server .pgm/.yaml
//
// FAST-LIO2 (external, not modified here) writes its accumulated cloud with
// pcl::PCDWriter::writeBinary, i.e. an *uncompressed binary* PCD of
// pcl::PointXYZINormal. This tool loads it with pcl::io::loadPCDFile, which
// transparently handles ASCII, binary and binary_compressed PCD and maps the
// x/y/z fields into pcl::PointXYZ (extra fields such as intensity/normals are
// ignored), then projects the points onto a horizontal grid:
//
//   * occupied  - >= --occupied-threshold points fall in the obstacle height
//                 slab [--z-min, --z-max]           (walls, furniture, ...)
//   * free      - not occupied AND >= --free-threshold points fall in the
//                 ground band [--ground-z-min, --z-min)   (observed floor)
//   * unknown   - no evidence either way; never guessed
//
// Outputs <output-dir>/<name>.pgm (8-bit binary P5, map_server greyscale
// convention) and <name>.yaml. It never overwrites an existing file.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace {

const int EXIT_OK = 0;
const int EXIT_USAGE = 1;
const int EXIT_VALIDATION = 2;
const int EXIT_EXISTS = 3;
const int EXIT_PCD = 4;
const int EXIT_DEGENERATE = 5;
const int EXIT_IO = 6;

// map_server greyscale convention (identical to map_saver output).
const unsigned char PGM_OCCUPIED = 0;
const unsigned char PGM_FREE = 254;
const unsigned char PGM_UNKNOWN = 205;

struct Options {
  std::string input;
  std::string name;
  std::string output_dir;
  double resolution = 0.05;
  double z_min = -0.5;
  double z_max = 2.0;
  double ground_z_min = std::numeric_limits<double>::quiet_NaN();  // default: z_min - 1.0
  double padding = 1.0;
  int occupied_threshold = 1;
  int free_threshold = 1;
  int negate = 0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
  double max_megapixels = 25.0;
};

void printUsage(std::ostream& os) {
  os <<
    "Usage: rosrun g1_slam pcd_to_map --input <map.pcd> --name <basename> [options]\n"
    "\n"
    "Offline converter: FAST-LIO2 PCD map -> ROS map_server .pgm + .yaml.\n"
    "Reads ASCII, binary and binary_compressed PCD.\n"
    "\n"
    "Required:\n"
    "  --input PATH            input PCD file (any PCL-readable format)\n"
    "  --name NAME             output basename, no path or extension\n"
    "                         (allowed: letters, digits, '.', '_', '-';\n"
    "                          no leading '.', no '..')\n"
    "\n"
    "Options:\n"
    "  --output-dir DIR        output directory (default: <g1_slam>/maps)\n"
    "  --resolution M          metres per pixel (default 0.05)\n"
    "  --z-min M               bottom of the obstacle height slab (default -0.5)\n"
    "  --z-max M               top of the obstacle height slab (default 2.0)\n"
    "  --ground-z-min M        bottom of the ground band (default: z-min - 1.0)\n"
    "  --padding M             free border added around the cloud (default 1.0)\n"
    "  --occupied-threshold N  min slab points in a cell -> occupied (default 1)\n"
    "  --free-threshold N      min ground points in a cell -> free (default 1)\n"
    "  --negate 0|1            map_server 'negate' field, written to yaml (default 0)\n"
    "  --occupied-thresh F     map_server 'occupied_thresh' for yaml (default 0.65)\n"
    "  --free-thresh F         map_server 'free_thresh' for yaml (default 0.196)\n"
    "  --max-megapixels F      refuse to allocate a larger grid (default 25)\n"
    "  --help                  show this help and exit\n";
}

bool parseDouble(const std::string& s, double* out) {
  try {
    size_t idx = 0;
    double v = std::stod(s, &idx);
    if (idx != s.size()) return false;
    *out = v;
    return true;
  } catch (...) {
    return false;
  }
}

bool parseInt(const std::string& s, int* out) {
  try {
    size_t idx = 0;
    int v = std::stoi(s, &idx);
    if (idx != s.size()) return false;
    *out = v;
    return true;
  } catch (...) {
    return false;
  }
}

bool isValidName(const std::string& n) {
  if (n.empty() || n.size() > 128) return false;
  if (n[0] == '.') return false;                     // no hidden files, no '.'/'..'
  if (n.find("..") != std::string::npos) return false;
  for (char c : n) {
    const bool ok = std::isalnum(static_cast<unsigned char>(c)) ||
                    c == '.' || c == '_' || c == '-';
    if (!ok) return false;                           // rejects '/', spaces, etc.
  }
  return true;
}

bool isDir(const std::string& p) {
  struct stat st;
  return stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}
bool isFile(const std::string& p) {
  struct stat st;
  return stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}
bool pathExists(const std::string& p) {
  struct stat st;
  return stat(p.c_str(), &st) == 0;
}

std::string toLower(std::string s) {
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

std::string baseName(const std::string& p) {
  const size_t s = p.find_last_of('/');
  return s == std::string::npos ? p : p.substr(s + 1);
}

bool endsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  const std::vector<std::string> args(argv + 1, argv + argc);

  for (size_t i = 0; i < args.size(); ++i) {
    const std::string& a = args[i];
    auto need = [&](void) -> std::string {
      if (i + 1 >= args.size()) {
        std::cerr << "error: " << a << " requires a value\n";
        std::exit(EXIT_USAGE);
      }
      return args[++i];
    };
    auto num = [&](double* dst) {
      const std::string v = need();
      if (!parseDouble(v, dst)) {
        std::cerr << "error: " << a << " expects a number, got '" << v << "'\n";
        std::exit(EXIT_USAGE);
      }
    };
    auto inum = [&](int* dst) {
      const std::string v = need();
      if (!parseInt(v, dst)) {
        std::cerr << "error: " << a << " expects an integer, got '" << v << "'\n";
        std::exit(EXIT_USAGE);
      }
    };

    if (a == "--help" || a == "-h") {
      printUsage(std::cout);
      return EXIT_OK;
    } else if (a == "--input") {
      o.input = need();
    } else if (a == "--name") {
      o.name = need();
    } else if (a == "--output-dir") {
      o.output_dir = need();
    } else if (a == "--resolution") {
      num(&o.resolution);
    } else if (a == "--z-min") {
      num(&o.z_min);
    } else if (a == "--z-max") {
      num(&o.z_max);
    } else if (a == "--ground-z-min") {
      num(&o.ground_z_min);
    } else if (a == "--padding") {
      num(&o.padding);
    } else if (a == "--occupied-threshold") {
      inum(&o.occupied_threshold);
    } else if (a == "--free-threshold") {
      inum(&o.free_threshold);
    } else if (a == "--negate") {
      inum(&o.negate);
    } else if (a == "--occupied-thresh") {
      num(&o.occupied_thresh);
    } else if (a == "--free-thresh") {
      num(&o.free_thresh);
    } else if (a == "--max-megapixels") {
      num(&o.max_megapixels);
    } else {
      std::cerr << "error: unknown argument: " << a << "\n\n";
      printUsage(std::cerr);
      return EXIT_USAGE;
    }
  }

  // ---- validate arguments -------------------------------------------------
  if (o.input.empty() || o.name.empty()) {
    std::cerr << "error: --input and --name are required\n\n";
    printUsage(std::cerr);
    return EXIT_USAGE;
  }
  if (!isValidName(o.name)) {
    std::cerr << "error: invalid --name '" << o.name << "': allowed characters are "
                 "letters, digits, '.', '_', '-'; no path separators, no leading "
                 "'.', no '..'\n";
    return EXIT_VALIDATION;
  }
  if (std::isnan(o.ground_z_min)) o.ground_z_min = o.z_min - 1.0;

  if (o.output_dir.empty()) {
    o.output_dir = ros::package::getPath("g1_slam");
    if (o.output_dir.empty()) {
      std::cerr << "error: could not resolve the g1_slam package for the default "
                   "output directory; pass --output-dir explicitly\n";
      return EXIT_VALIDATION;
    }
    o.output_dir += "/maps";
  }
  // Normalise a single trailing slash away for tidy messages/paths.
  while (o.output_dir.size() > 1 && o.output_dir.back() == '/') o.output_dir.pop_back();

  if (!isFile(o.input)) {
    std::cerr << "error: --input is not a readable file: " << o.input << "\n";
    return EXIT_VALIDATION;
  }
  if (!endsWith(toLower(o.input), ".pcd")) {
    std::cerr << "error: --input must have a .pcd extension: " << o.input << "\n";
    return EXIT_VALIDATION;
  }
  if (!isDir(o.output_dir)) {
    std::cerr << "error: --output-dir is not a directory: " << o.output_dir << "\n";
    return EXIT_VALIDATION;
  }
  if (access(o.output_dir.c_str(), W_OK) != 0) {
    std::cerr << "error: --output-dir is not writable: " << o.output_dir << "\n";
    return EXIT_VALIDATION;
  }
  if (!(o.resolution > 1e-4)) {
    std::cerr << "error: --resolution must be > 0.0001 m/pixel\n";
    return EXIT_VALIDATION;
  }
  if (!(o.z_min < o.z_max)) {
    std::cerr << "error: --z-min (" << o.z_min << ") must be < --z-max (" << o.z_max << ")\n";
    return EXIT_VALIDATION;
  }
  if (o.ground_z_min > o.z_min) {
    std::cerr << "error: --ground-z-min (" << o.ground_z_min << ") must be <= --z-min ("
              << o.z_min << ")\n";
    return EXIT_VALIDATION;
  }
  if (o.padding < 0.0) {
    std::cerr << "error: --padding must be >= 0\n";
    return EXIT_VALIDATION;
  }
  if (o.occupied_threshold < 1 || o.free_threshold < 1) {
    std::cerr << "error: --occupied-threshold and --free-threshold must be >= 1\n";
    return EXIT_VALIDATION;
  }
  if (o.negate != 0 && o.negate != 1) {
    std::cerr << "error: --negate must be 0 or 1\n";
    return EXIT_VALIDATION;
  }
  if (!(o.free_thresh > 0.0 && o.free_thresh < o.occupied_thresh && o.occupied_thresh < 1.0)) {
    std::cerr << "error: require 0 < --free-thresh < --occupied-thresh < 1 (got "
              << o.free_thresh << ", " << o.occupied_thresh << ")\n";
    return EXIT_VALIDATION;
  }
  if (!(o.max_megapixels > 0.0)) {
    std::cerr << "error: --max-megapixels must be > 0\n";
    return EXIT_VALIDATION;
  }

  const std::string pgm_path = o.output_dir + "/" + o.name + ".pgm";
  const std::string yaml_path = o.output_dir + "/" + o.name + ".yaml";
  if (pathExists(pgm_path) || pathExists(yaml_path)) {
    std::cerr << "error: refusing to overwrite an existing map (" << pgm_path << " or "
              << yaml_path << "); choose a different --name or --output-dir\n";
    return EXIT_EXISTS;
  }

  // ---- load the cloud ---------------------------------------------------
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(o.input, cloud) < 0) {
    std::cerr << "error: failed to load PCD (not a valid point cloud, or missing "
                 "x/y/z fields): " << o.input << "\n";
    return EXIT_PCD;
  }
  if (cloud.empty()) {
    std::cerr << "error: PCD contains no points: " << o.input << "\n";
    return EXIT_DEGENERATE;
  }

  // ---- XY bounds over finite points -----------------------------------
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  size_t finite = 0;
  for (const auto& p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    ++finite;
    min_x = std::min(min_x, static_cast<double>(p.x));
    max_x = std::max(max_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_y = std::max(max_y, static_cast<double>(p.y));
  }
  if (finite == 0) {
    std::cerr << "error: PCD has no finite XYZ points: " << o.input << "\n";
    return EXIT_DEGENERATE;
  }

  min_x -= o.padding;
  min_y -= o.padding;
  max_x += o.padding;
  max_y += o.padding;

  const long width = static_cast<long>(std::ceil((max_x - min_x) / o.resolution));
  const long height = static_cast<long>(std::ceil((max_y - min_y) / o.resolution));
  if (width < 1 || height < 1) {
    std::cerr << "error: computed an empty grid; check --padding / --resolution\n";
    return EXIT_DEGENERATE;
  }
  const double megapixels = static_cast<double>(width) * static_cast<double>(height) / 1e6;
  if (megapixels > o.max_megapixels) {
    std::cerr << "error: grid would be " << width << " x " << height << " ("
              << std::fixed << std::setprecision(3) << megapixels
              << " Mpix) which exceeds --max-megapixels " << o.max_megapixels
              << "; use a coarser --resolution or tighter bounds\n";
    return EXIT_DEGENERATE;
  }

  // ---- accumulate -----------------------------------------------------
  const size_t cells = static_cast<size_t>(width) * static_cast<size_t>(height);
  std::vector<uint32_t> slab(cells, 0);
  std::vector<uint32_t> ground(cells, 0);
  for (const auto& p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const long col = static_cast<long>(std::floor((p.x - min_x) / o.resolution));
    const long row = static_cast<long>(std::floor((max_y - p.y) / o.resolution));  // row 0 = north
    if (col < 0 || col >= width || row < 0 || row >= height) continue;
    const size_t idx = static_cast<size_t>(row) * static_cast<size_t>(width) +
                       static_cast<size_t>(col);
    if (p.z >= o.z_min && p.z <= o.z_max) {
      slab[idx] += 1;
    } else if (p.z >= o.ground_z_min && p.z < o.z_min) {
      ground[idx] += 1;
    }
  }

  // ---- classify -----------------------------------------------------
  std::vector<unsigned char> pgm(cells, PGM_UNKNOWN);
  size_t n_occ = 0, n_free = 0, n_unknown = 0;
  for (size_t i = 0; i < cells; ++i) {
    if (slab[i] >= static_cast<uint32_t>(o.occupied_threshold)) {
      pgm[i] = PGM_OCCUPIED;
      ++n_occ;
    } else if (ground[i] >= static_cast<uint32_t>(o.free_threshold)) {
      pgm[i] = PGM_FREE;
      ++n_free;
    } else {
      pgm[i] = PGM_UNKNOWN;
      ++n_unknown;
    }
  }
  if (n_occ == 0 && n_free == 0) {
    std::cerr << "error: no cell was classified as occupied or free; check --z-min / "
                 "--z-max / --ground-z-min against your map's actual height range\n";
    return EXIT_DEGENERATE;
  }

  // ---- write .pgm (binary P5) --------------------------------------
  {
    std::ofstream f(pgm_path.c_str(), std::ios::binary | std::ios::trunc);
    if (!f) {
      std::cerr << "error: cannot create " << pgm_path << "\n";
      return EXIT_IO;
    }
    f << "P5\n"
      << "# g1_slam pcd_to_map from " << baseName(o.input) << " @ " << o.resolution
      << " m/pix, obstacle z [" << o.z_min << "," << o.z_max << "]\n"
      << width << " " << height << "\n255\n";
    f.write(reinterpret_cast<const char*>(pgm.data()), static_cast<std::streamsize>(pgm.size()));
    f.flush();
    if (!f) {
      std::cerr << "error: failed while writing " << pgm_path << "\n";
      std::remove(pgm_path.c_str());
      return EXIT_IO;
    }
  }

  // ---- write .yaml ------------------------------------------------
  {
    std::ofstream f(yaml_path.c_str(), std::ios::trunc);
    if (!f) {
      std::cerr << "error: cannot create " << yaml_path << "\n";
      std::remove(pgm_path.c_str());
      return EXIT_IO;
    }
    f << std::fixed << std::setprecision(6);
    f << "image: " << o.name << ".pgm\n"
      << "resolution: " << o.resolution << "\n"
      << "origin: [" << min_x << ", " << min_y << ", 0.000000]\n"
      << "negate: " << o.negate << "\n"
      << "occupied_thresh: " << o.occupied_thresh << "\n"
      << "free_thresh: " << o.free_thresh << "\n";
    f.flush();
    if (!f) {
      std::cerr << "error: failed while writing " << yaml_path << "\n";
      std::remove(pgm_path.c_str());
      std::remove(yaml_path.c_str());
      return EXIT_IO;
    }
  }

  std::cout << "wrote " << pgm_path << "\n"
            << "      " << yaml_path << "\n"
            << "  grid:   " << width << " x " << height << " px @ " << o.resolution
            << " m/px\n"
            << "  origin: [" << std::fixed << std::setprecision(3) << min_x << ", "
            << min_y << ", 0]\n"
            << "  cells:  " << n_occ << " occupied, " << n_free << " free, "
            << n_unknown << " unknown\n"
            << "  points: " << finite << " finite / " << cloud.size() << " total\n";
  return EXIT_OK;
}
