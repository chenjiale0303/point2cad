# Point2CAD Evaluation

## **HPNet+Point2CAD**

To obtain data for evaluation of **HPNet+Point2CAD**, you should follow these steps:

1.  Input your test data into **HPNet** to generate xyzc files. Specially, write the model ids you want to generate in the file named `test_ids.txt`, with each model's id on a new line.
2.  Run `python test.py` to generate the `out` directory containing `.xyzc` files.
3.  Use the generated xyzc files as input for **Point2CAD** to obtain the corresponding mesh. Run `python extract_mesh.py --output OUTPUT_PATH`.
4.  Sample points based on area on the transformed mesh, resulting in `mesh_transformed_sampled.ply`. Specially, complie c++ `prepare_data_for_point2CAD_evaluate` code, and run 
```cmd
cd build &  
./src/GSP_Field/prepare_data_for_point2CAD_evaluate/prepare_data_for_point2CAD_evaluate POINT2CAD_ROOT/OUTPUT_PATH POINT2CAD_ROOT/OUTPUT_PATH
```
5. **(Optional)** You can implement your own sampling code in `Python`, but note that you need to refer to the colors in the `color.txt` file. Each color represents the index of a specific primitive. The output of `point2cad` is colored according to this file. You need to assign a primitive index to the color of each triangular facet for subsequent evaluation.

Complie c++ evaluation code, and run:
```cmd
cd build & 
./src/GSP_Field/evaluate/evaluate /data/baselines/HP_mesh_0409 /data/test_data_whole3/gt /data/baselines/misc/HP_eval_0409 --point2CAD_input --chamfer --matched --dist -1
```

## **SEDNet+Point2CAD**

Similar to HPNet(The instructions running are the same), to obtain data for evaluation of **SEDNet+Point2CAD**, you should follow these steps:

1.  Input your test data into **SEDNet** to generate xyzc files.
2.  Run `python test.py` to generate the `out` directory containing `.xyzc` files.
3.  Use the generated xyzc files as input for **Point2CAD** to obtain the corresponding mesh. Run `python extract_mesh.py --output OUTPUT_PATH`.
4.  Sample points based on area on the transformed mesh, resulting in `mesh_transformed_sampled.ply`. Specially, complie c++ `prepare_data_for_point2CAD_evaluate` code, and run 

Complie c++ evaluation code, and run:
```cmd
cd build & 
./src/GSP_Field/evaluate/evaluate /data/baselines/SED_mesh_0413 /data/test_data_whole3/gt /data/baselines/misc/SED_eval_0413 --point2CAD_input --chamfer --matched --dist -1
```