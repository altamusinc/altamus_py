# Introduction

This library contains mid-level tools for interacting with the Altamus EOS2 Scanner. It covers functions such as:
- Parse the binary files that the scanners generate and export them to .pcd files
- Parse .pcd files and turn them back into binary files
- Mavlink functions to send/received messages from the scanner

## Messaging

The EOS2 communicates over MAVLink.

Altamus has a fork of the MAVLink project that contains the Altamus dialect. The messages are defined in `/mavlink/message_definitions/v1.0/altamus.xml`

## Building

Be sure to initialize the Mavlink definition submodule. `git submodule update --init --recursive`

run `pip install -e .` to have pip install a version that updates as changes are made to the source files

otherwise run `pip install .` to install the wheel normally

## Updating Mavlink

If the upstream MAVLink definitions have changed update them with `git submodule update --remote`. This will pull the latest definitions from master. 

Then run `update_mavlink.sh` 

This will re-generate the python file that is used for actually sending/receiving the messages. It's located at `src/altamus_py/mavlink.py`

## Pushing to PyPi

We use Twine to publish to PyPi (pip) which distributes the package.

Create a file called `token.env` at the root of the project. Enter your token as a variable called `TWINE_TOKEN`
e.g., `TWINE_TOKEN="replace with real token"`

Update the version in `pyproject.toml` to a unique one

Run `build_and_push.sh`