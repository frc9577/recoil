# recoil
This is the main repository for all software produced by FRC 9577 "Recoil" from Dripping Springs, Texas.

## Repository Structure

The `development` and `main` branches hold only the current season's competition code. Everything else lives on its own branch, listed below.

### 2026-Rebuilt

The competition code for the 2026 season. The directory contains a single Java project that, when built, is the code that runs on the robot. CI builds this directory.

### Where the other projects went

| Was | Now | Notes |
|---|---|---|
| `2024-Crescendo/` | branch `archive/2024-crescendo` | Snapshot of robot-code-2024. The project sits at the branch root. |
| `2025-Reefscape/` | branch `archive/2025-reefscape` | Snapshot of robot-code-2025, with Examples/PrototypeBoard and Examples/Shooter. The project sits at the branch root. |
| `prototype_projects/` | branch `prototypes` | Off-season prototypes, examples and experiments. Each top-level directory is one project. `2027 Alpha` is next season's starter. Code there is not guaranteed to build. |

These branches share no commits with `development` or `main`. Each keeps the commits that touched its tree. The 2024 and 2025 trees were imported as snapshots, so their older history is in the original repos. Never open a PR from these branches into `development` or `main`. Git refuses that merge, and a forced merge would delete `2026-Rebuilt`.

To look at an archived season: `git switch archive/2025-reefscape`. WPILib opens the branch root as the project.

To add or change a prototype: branch from `prototypes`, make the change, and open a PR with base `prototypes`. GitHub sets the PR base to `main` by default, so change it. The feature-branch rules below are for competition code only.

At the end of a season, archive the competition directory the same way and start the next season's directory on `development` from the starter on `prototypes`:

```
git branch archive/<year>-<game> development
git filter-branch --subdirectory-filter <year>-<game> archive/<year>-<game>
git push origin archive/<year>-<game>
git rm -r <year>-<game>        # on a PR branch into development, and again on one into main
```

## Branching Strategy and Commit Rules

This repository uses a modified feature branch workflow. Where all big features are done on short-lived feature branches, and then merged into a development branch for final bug testing before being put through pull request (PR) into main. Although, if the change is quick enough to do in one setting and meets the commit requirements (talked about later) for the development branch the change can be committed directly to it.

### Main: 
The most stable copy of the current year's competition code. Code cannot be directly committed to this branch and instead it is required to use a PR. Before a PR is merged into main, a full test of the robot is required to make sure every feature works. Automation runs the build on PRs to main and build failures block the ability to merge. After a merge, the build also runs on main.

### Development: 
This branch houses all of the in-development features that aren't stable enough for main. Although, we should try our best to make sure that the current year's code in the development branch always works via quick testing on a mule or similar to make sure that any changes do not outright “brick” the robot. Pull requests are not required, but if you think a change should have a discussion feel free to make one. During a PR to development, a build is run by automation but a build failure is not blocking. Pushes to development also trigger builds.

### Feature Branches: 
Feature branches are short-lived branches to work on specific features, they should be named with this format: “YEAR-FEATURE_SHORTHAND”. An example is “2026-ground_pickup” which would be a feature branch for a ground pickup for the 2026 competition. These branches have no commit requirements and are not required to run or build. This is so we can upload our changes to the github at the end of sessions without losing it due to the chance of using a different computer. After a feature is completed, make sure to merge development into your branch and make sure that the robot still mostly works via a quick test. Once that is completed you can merge your branch into development and delete the feature branch.
