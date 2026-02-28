# Contributing to ENFIELD

Thank you for your interest in contributing to the ENFIELD project. This document outlines the guidelines for contributing code, documentation, and other improvements.

## Project Context

This is a research project under the ENFIELD Exchange Scheme (Horizon Europe, Grant Agreement No 101120657). Contributions must align with the project's Open Science requirements and simulation-only scope.

## Development Setup

```bash
# Clone
git clone https://github.com/dryuemco/industrial_robot_security.git
cd industrial_robot_security

# Docker (recommended)
docker compose build
docker compose run --rm dev

# Or native ROS2 Humble
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install
```

## Branch Strategy

- `main` — stable, CI-passing, release-ready
- `develop` — integration branch for PRs
- `feature/<name>` — feature branches (e.g., `feature/ur5e-adapter`)
- `fix/<name>` — bug fixes
- `docs/<name>` — documentation-only changes

## Commit Convention

We follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <short description>

[optional body]

[optional footer]
```

**Types:** `feat`, `fix`, `docs`, `test`, `ci`, `refactor`, `chore`, `build`

**Scopes:** `core`, `ur5e`, `tasks`, `watchdog`, `docker`, `ci`, `open-science`

**Examples:**
```
feat(ur5e): add Gazebo launch file with ros2_control
fix(watchdog): correct velocity threshold comparison
docs(open-science): add OSF preregistration draft
test(core): add unit tests for IR validator
ci: add SBOM generation step to workflow
chore: update .gitignore for colcon artifacts
```

## Pull Request Process

1. Create a feature branch from `develop`
2. Keep PRs small and focused (one concern per PR)
3. Ensure `colcon build` and `colcon test` pass locally
4. Include or update tests for new functionality
5. Update documentation if behavior changes
6. Fill in the PR template (objectives, files changed, test results)
7. Request review from project maintainers

## Code Standards

### Python
- Type hints on all public functions
- Docstrings (Google style) on all modules, classes, and public functions
- `logging` module instead of `print()` statements
- Configurable parameters via YAML, not hardcoded values
- Deterministic seed for any randomized operations

### ROS2
- Use `ament_python` or `ament_cmake` build type as appropriate
- Launch files in Python (`.launch.py`)
- Parameters via YAML config files
- Proper `package.xml` dependencies declared

### Testing
- Unit tests with `pytest`
- Integration tests for ROS2 nodes (launch_testing where applicable)
- Minimum 80% line coverage for new code
- All tests must pass in Docker environment

## Open Science Requirements

All contributions must respect the project's Open Science obligations:

- **License:** All code must be compatible with Apache-2.0
- **No proprietary dependencies:** All dependencies must be OSI-licensed
- **Reproducibility:** Any new functionality must be runnable inside Docker
- **Documentation:** New features require corresponding docs updates
- **ISO traceability:** Safety-related changes must reference ISO 10218 clauses

## Safety Scope

This project operates exclusively in simulation. Contributions must NOT:

- Include instructions for deploying to physical robots
- Contain real-world exploit code beyond simulation test purposes
- Remove or weaken safety monitoring without documented justification

## Reporting Issues

Use GitHub Issues with the following labels:
- `bug` — something is broken
- `enhancement` — new feature request
- `documentation` — docs improvement
- `open-science` — OSF/DOI/license/replication concerns
- `question` — discussion needed

## Contact

- **Researcher:** Yunus Emre Cogurcu (yunusemrecogurcu@gmail.com)
- **Supervisor:** Assoc. Prof. Georgios Spathoulas (NTNU)
