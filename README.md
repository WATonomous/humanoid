# wato_asd_training
WATonomous ASD Admissions Assignment

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize Rootless docker to enable ease of reproducibility and deployability.

> Why rootless docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. [Download Rootless Docker](https://docs.docker.com/engine/security/rootless/#install)
4. [Setup Rootless Docker to start on boot](https://docs.docker.com/engine/security/rootless/#usage)
5. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

> Make sure you are always running the assignment inside Linux

Link to Onboarding Assignment: https://wiki.watonomous.ca/
