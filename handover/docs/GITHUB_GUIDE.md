# Uploading the Handover Package to GitHub

This guide describes the workflow using GitHub Desktop. Interface labels may vary between versions.

## Basic terms

| Term | Meaning |
|---|---|
| Repository | The project's files and change history |
| Clone | Download a repository together with its history |
| Branch | A separate line of work for your changes |
| Commit | Save a recorded set of changes with a description |
| Push | Upload your commits to GitHub |
| Fork | A copy of another repository under your own account |
| Pull request | Ask the team to review and merge your branch |

## Prepare

You need a GitHub account, GitHub Desktop, and either write access to the team repository or your own fork. Ask the repository owner to add you as a collaborator, or fork the repository into your account.

Team repository: https://github.com/Haosheng333/g1

## Clone and create a branch

Use GitHub Desktop to clone the team repository, or your fork if you do not have write access. Choose a new local folder, such as `Documents/DIP/g1`, to distinguish it from the downloaded Desktop folder.

Create a branch named `handover-implementation`.

## Copy the package

Copy only the delivered `handover` folder into the cloned repository root, next to `navigation`, `arm_control`, and `hand_control`.

```text
g1/
├── navigation/
├── arm_control/
├── hand_control/
└── handover/
    ├── README.md
    ├── scripts/
    ├── srv/
    └── ...
```

Do not nest the entire delivered `g1-handover` repository inside the cloned repository. If the team's handover folder has changed since the supplied snapshot, compare and merge those changes before replacing files.

The package includes a `.gitignore` for Python caches. Do not upload `__pycache__`, `.pyc`, build outputs, credentials, or unrelated files. Existing model and map files do not need to be uploaded again for this change.

## Review, commit, and push

Review the changed files in GitHub Desktop. They should be confined to `handover/` for this delivery.

Suggested commit summary:

```text
Add tactile handover module and mock demo
```

Commit to your branch, then publish or push the branch to GitHub.

## Open a pull request

Open a pull request from your branch to the team's main branch. If using a fork, make sure the destination is the original team repository.

Suggested title:

```text
Add tactile handover coordinator and mock demonstration
```

Suggested description:

```text
Adds adaptive tactile change detection, handover sequencing, and mock dependencies.

Validation: five pure Python detector tests pass; Python syntax checks pass.
ROS build, ROS integration, model loading, and physical robot tests remain unverified.

Required integration:
- Implement the /arm/plan_handover adapter with validated paths.
- Publish real tactile data using the documented message contract.
- Integrate verified grasp state, navigation standstill, and global execution locking.
```

Ask the arm, hand, and orchestration owners to review the interface contracts before merging.
