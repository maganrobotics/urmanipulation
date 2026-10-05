# Environment Notes — Community v0.1

The repository currently contains a historical pinned `requirements.txt` from the original development environment. It includes a broad Jupyter/scientific stack and Windows-specific packages such as `pywin32`.

For that reason, **do not interpret `requirements.txt` as a promise that every modern Python/OS combination installs cleanly**.

## Conservative setup

For the first Community release:

1. Use an isolated virtual environment.
2. Keep the original `requirements.txt` unchanged for reproducibility/history.
3. Start from simulation-only examples.
4. Record the exact Python version and OS when an example is confirmed working.
5. Do not connect a physical robot merely because a simulation launches successfully.

## Dependency cleanup plan

A later Community update should split dependencies into smaller groups, for example:

```text
core / simulation
vision
notebooks
hardware
development
```

This should be done only after representative examples have been tested so dependency cleanup does not silently break upstream functionality.

## Known v0.1 limitations

- The dependency set dates from an older Python ecosystem.
- Some examples use relative paths and assume a particular working directory.
- Some application examples depend on assets, datasets, or hardware-specific context.
- Example verification is being added incrementally.

These limitations are documented rather than hidden so learners can distinguish an algorithm example from a supported deployment configuration.
