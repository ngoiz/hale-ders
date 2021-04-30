# HALE Aircraft Analysis Using SHARPy

This repository contains examples and tutorials of new SHARPy capabilities applied to a HALE aircraft. This
repository contains the needed packages as submodules. These submodules are:

- `src/model-hale`: The aircraft model.
- `src/sharpy/`: SHARPy at the corresponding version (may not yet be a released version).
- `src/analysis/`: utilities for postprocessing and managing SHARPy output data.

The finished examples that are ready for use are:

- `0)HALE Aircraft`: A tutorial on setting up the HALE aircraft class to obtain the `.fem.h5` and `.aero.h5` files
  used in SHARPy.
  
- `Delivery/01_StabilityDerivatives`; Tutorial going through the required settings in order to obtain the aerodynamic
  and aeroelastic stability derivatives of the HALE aircraft.
  
## Installation

To install this repository, simply clone it recursively, to get the submodules too.

```bash
git clone http://github.com/ngoiz/hale-ders -r
```

Then, initialise and update the submodules:

```bash
git submodule update --init --recursive
```

Load the the variables (for SHARPy, the model and the analysis package)

```bash
source src/sharpy/bin/sharpy_vars.sh
source src/model-hale/bin/hale_vars.sh
source src/analysis/bin/analysis_vars.sh
```

and install SHARPy as instructued (including the `conda` environment and compiling both `UVLM` and `xbeam`).

You should now be able to run these notebooks using simply
```bash
jupyter lab
```