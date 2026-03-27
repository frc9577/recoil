# robot-base
FRC 9577's Template Repository for Robot Code

# How to use this template
This repositry is a github template repository. It represents the baseline code that FRC 9577 uses for our robots, including dependencies we use in our designs. To use our code, create a repository using this one as a template.

## Repository Approach
The approach here is to minimize setup time for a developer. By leveraging the WPILib distributed docker container and through effective use of docker we limit the prerequisite install to just a few items. This approach is for development only and does not replace the driver station. We support both linux and Windows for development. (Currently )

## Installing Prerequsiites
This repository does not contain any tools. You need to install four tools to support your build: git, VSCode, make, and docker.

### Installing on Linux
To install on linux, use your package manager to pick up git, make, and docker. Testing was done on git 2.43, make 4.3, and docker 27.3.1.

Both make and git come directly from the package manager (e.g. for ubuntu `apt install make git`).

VS Code is installation instructions are here: https://code.visualstudio.com/docs/setup/linux

Docker installation instructions are here: https://docs.docker.com/engine/install/ubuntu/

### Installing on Windows
Herein, we provide references to each of the windows installers:https://gnuwin32.sourceforge.net/packages/make.htmz
* git: https://git-scm.com/downloads/win
* make: https://gnuwin32.sourceforge.net/packages/make.htm
* docker: https://docs.docker.com/desktop/install/windows-install/
* VSCode: https://code.visualstudio.com/docs/setup/windows

This is more manual than a linux install, but it's just 4 tools.

When installing git, choose the option to install bash. Always use bash as your windows terminal when using this repo, as we only test on bash.

### Windows "funnies"
- Need to install make for everyone and add the make path manually to system paths.
- Need to build via external git bash terminal started as administrator.

# Updating this template
When a new version of tools or libraries becomes available, use the WPILIB VSCode plugin to update the files in the project. Then, open a pull request against main to merge your change in.

Sometimes there are updates not serviced by the WPILib plugin. In these cases, make the changes using editors and push the results to github.

This repository can also be forked.
