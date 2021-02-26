# FOWT Linear Analysis
These scripts can be useful for generating and analyzing linear models of FOWT wind turbines. 

## Necessary tools:
- OpenFAST: https://github.com/OpenFAST/openfast
- matlab_toolbox: https://github.com/dzalkind/matlab-toolbox/
 
OpenFAST is needed to generate linear models. The matlab_toolbox is used to post-process these linear models and run the MBC transform. We use Dan Zalkind's fork of the matlab toolbox to leaverage some ROSCO-related tools that he has developed.

## Setup
You will need to pre-emptively generate a lot of OpenFAST linear models. These are loaded at the beginning of `system_analysis.m`. Generally, a number of linearizations should be done accross a range of wind speeds (e.g. rated -> cutout), at 12 or so Azimuth positions. Some example linearizations are provided in the `iea15mw_cases` folder.
