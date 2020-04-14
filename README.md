# Cassie Description
Package containing MATLAB and C++ model files for a compliant model of the Cassie biped.

## Generating source files
We do not commit all of the codegen to the repository. Before this package is used, you must either follow these instructions for codegen, or [download and extract some pregenerated code](https://www.dropbox.com/s/ff3dfvctna8amwy/cassie_description_pregen.zip?dl=0) (this will mean that you cannot configure the model further yourself). If you use the pregenerated code, simply extract the archive to the root directory of this package.

You must have MATLAB and Mathematica installed to generate the files yourself.

First, extract and install the [FROST (Fast Robot Optimization and Simulation Toolkit) package](https://github.com/ayonga/frost-dev) according to the [installation wiki](https://ayonga.github.io/frost-dev/pages/installation.html). 

Then, simply run `generate_libraries.m` from the root of this repository.


## Related Literature:
The models used herein are based on the results presented in our ECC publication, and drawn from the existing files and specifications from Agility Robotics.
* Reher, Jenna, Wen-Loong Ma, and Aaron D. Ames. "Dynamic walking with compliance on a cassie bipedal robot." 2019 18th European Control Conference (ECC). IEEE, 2019.
```
@inproceedings{reher2019dynamic,
  title={Dynamic walking with compliance on a {Cassie} bipedal robot},
  author={Reher, Jenna and Ma, Wen-Loong and Ames, Aaron D},
  booktitle={2019 18th European Control Conference (ECC)},
  pages={2589--2595},
  year={2019},
  organization={IEEE}
}
```

* The official Agility Robotics documentation and software release for the Cassie biped.

https://github.com/agilityrobotics/agility-cassie-doc
