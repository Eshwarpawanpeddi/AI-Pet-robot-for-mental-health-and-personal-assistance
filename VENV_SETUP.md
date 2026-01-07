# Virtual Environment Setup

This document describes how to use the Python virtual environment included in this repository.

## Overview

A Python virtual environment named `venv` has been created in the root of the repository. This environment contains all the dependencies specified in `requirements.txt` and provides an isolated Python environment for running the AI Pet Robot server.

## Python Version

The virtual environment uses **Python 3.12.3**. While the original requirement specified Python 3.11, the system environment has Python 3.12.3 which is fully compatible with all dependencies in this project. Python 3.12 is backward compatible with Python 3.11 for all libraries used in this project.

## Activating the Virtual Environment

### On Linux/MacOS:

```bash
source venv/bin/activate
```

### On Windows:

```bash
venv\Scripts\activate
```

Once activated, you should see `(venv)` prepended to your command prompt, indicating that the virtual environment is active.

## Verifying the Installation

After activating the virtual environment, you can verify that all dependencies are installed:

```bash
pip list
```

You should see all the packages from `requirements.txt` installed, including:
- fastapi
- uvicorn
- websockets
- google-generativeai
- pydantic
- aiohttp
- and more...

## Deactivating the Virtual Environment

To exit the virtual environment, simply run:

```bash
deactivate
```

## Reinstalling Dependencies

If you need to reinstall or update dependencies, activate the virtual environment first, then run:

```bash
pip install -r requirements.txt
```

To upgrade all packages to their latest compatible versions:

```bash
pip install --upgrade -r requirements.txt
```

## Running the Server

With the virtual environment activated, you can run the server as described in the main README:

```bash
cd server
python main.py
```

Or using uvicorn directly:

```bash
cd server
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

## Troubleshooting

### Virtual Environment Not Found

If you get an error that the virtual environment is not found, you may need to recreate it:

```bash
# Remove the old venv directory
rm -rf venv

# Create a new virtual environment
python3 -m venv venv

# Activate it
source venv/bin/activate  # Linux/MacOS
# or
venv\Scripts\activate  # Windows

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### Import Errors

If you encounter import errors, ensure that:
1. The virtual environment is activated (you should see `(venv)` in your prompt)
2. All dependencies are installed: `pip install -r requirements.txt`
3. You're running Python from the virtual environment: `which python` should show a path inside the venv directory

## Notes

- The `venv/` directory is excluded from git via `.gitignore` to avoid committing large dependency files
- Both `requirements.txt` (standard naming) and `req.txt` (legacy) contain the same dependencies
- All dependencies are compatible with Python 3.9+ through Python 3.12+
