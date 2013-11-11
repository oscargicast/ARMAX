ARMAX
=====

Identifying a third-order system using the parametric model ARMAX.

Note: **fs = 30Hz** 


MATLAB COMMANDS
---------------
Naive solution using ARMX. This is just a test example 
using ARMAX(na = 3; nb = 2; nc = 1; nk = 1);
```matlab
>> base_armax
```

This script compare the structures listed bellow by default:
nn = [
    3 1 1 1; 
    2 2 2 2; 
    2 2 1 1;
    1 1 1 1;
];
If you want to change one structure just change a row. We enter the model 
to compare and *vuala!*:
```matlab
>> compare_structures
signal(gate/step/seno/rampa): seno
...
```

To validate all the identified models against a GATE input. Running the 
script we have to enter the ARMAX parameters.
```matlab
>> gate_for_all_identified_models
ARMAX(na,nb,nc,nk): [2 2 1 1]
....
```


MATLAB GUIDE
------------

```matlab
>> ident('matlab_ident')
```

LABVIEW WORKBENCH
-----------------
The data is generated on the file:
```
data/labview_sim_data
```



