# PRM* PROJECT

## Installation Guide

### Linux Ubuntu

1. Install [Docker](https://docs.docker.com/engine/install/ubuntu/)

2. Install [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extenstion in VS Code. Do not check *Execute In WSL* in DEV Container: settings

3. Check if docker group exists, if not: 

    3.1 Create the docker group
        `sudo groupadd docker`

    3.2 Add user to the docker group
        `sudo usermod -aG docker $USER`

    3.3 Log out and log back in so that your group membership is re-evaluated.

4. Use View->Command Palette... or Ctrl+Shift+P to open the command palette. Search for the command Dev Containers: Reopen in Container and execute it. This will build docker container for your. It will take a while - sit back or go for a coffee.

5. After docker setup, run bash script from folder utils:
    `bash /utils/env_setup.sh`
    
    It will install required dependencies and setup .bashrc