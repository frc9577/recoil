# recoil

This is the main repository for all software produced by FRC 9577 "Recoil" from Dripping Springs, Texas.

## Repository Structure

### [year]-[competition]

These directories contain competition code for our FRC robot in the given year. The directory contains a single Java project that, when built, represents the code that runs on the robot for that competition.

This repository was created at the start of the 2026 season so you will find full history for files in the "2026-Rebuilt" tree. The repo is also seeded with files from our 2024 and 2025 seasons but these are merely snapshots of the "main" and "development" branches from our prior repos, "robot-code-2024" and "robot-code-2025". Visit the original repositories if you need to find history information for these files.

### prototype_projects

This top-level directory contains prototype, example and experiment projects developed during off-season or while working on a competition robot. Each subdirectory contains a single Java project. Code in this area is not guaranteed to be complete or working.

## Branching Strategy and Commit Rules

This repository uses a modified feature branch workflow. Where all big features are done on short-lived feature branches, and then merged into a development branch for final bug testing before being put through pull request (PR) into main. Although, if the change is quick enough to do in one setting and meets the commit requirements (talked about later) for the development branch the change can be committed directly to it.

### Main:

The most stable copy of the current year's competition code. Code cannot be directly committed to this branch and instead it is required to use a PR. Before a PR is merged into main, a full test of the robot is required to make sure every feature works. Automation runs the build on PRs to main and build failures block the ability to merge. After a merge, the build also runs on main.

### Development:

This branch houses all of the in-development features that aren't stable enough for main. Although, we should try our best to make sure that the current year's code in the development branch always works via quick testing on a mule or similar to make sure that any changes do not outright “brick” the robot. Pull requests are not required, but if you think a change should have a discussion feel free to make one. During a PR to development, a build is run by automation but a build failure is not blocking. Pushes to development also trigger builds.

### Feature Branches:

Feature branches are short-lived branches to work on specific features, they should be named with this format: “YEAR-FEATURE_SHORTHAND”. An example is “2026-ground_pickup” which would be a feature branch for a ground pickup for the 2026 competition. These branches have no commit requirements and are not required to run or build. This is so we can upload our changes to the github at the end of sessions without losing it due to the chance of using a different computer. After a feature is completed, make sure to merge development into your branch and make sure that the robot still mostly works via a quick test. Once that is completed you can merge your branch into development and delete the feature branch.
