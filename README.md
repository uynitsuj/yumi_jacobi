# ABB YuMi IRB 14000 Jacobi Motion Wrapper

This YuMi wapper uses the **Jacobi Motion Library** for robot motion planning with Externally Guided Motion (EGM). Jacobi Motion provides a clean API for efficient algorithms that compute time-optimized, jerk-limited robot arm trajectories in milliseconds.

Primary reason for the wrapper is to include YuMi IRB 14000 SmartGripper functionality and interface. All motion planning functionality is taken care of by the Jacobi interface.

For instructions regrading installation, getting started, and for general documentation refer to [docs.jacobirobotics.com](https://docs.jacobirobotics.com).

## Installation
The Jacobi Cloud Python package can be installed from PyPI via

> pip install jacobi-motion

In case no package could be found, make sure to upgrade pip to the latest available version via pip install pip --upgrade.

## License
The Cloud version requires an API key for authentication that you can access at your user account at account.jacobirobotics.com. You can pass this to the motion library by setting the JACOBI_API_KEY and JACOBI_API_SECRET environment variable.