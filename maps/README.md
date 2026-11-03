# F1TENTH Gym Simulator Maps

This file explains how to make, modify, and change maps for the F1TENTH gym simulator.


## Creating/Modifying a New Map

1. Draw a map using an image editing software (MS Paint, Canva, etc.).
2. Add the image as a **PNG** in `./maps/<mapname>/`.
3. Convert the image to greyscale using **`greyscale.py`**.
4. Create a **`<mapname>.yaml`** file in the same directory as your map.
5. Copy the standard parameters below into the .yaml file and adjust them for your map:
   ```yaml
   image: <mapname>_greyscale.png
   resolution: 0.050000
   origin: [-51.224998, -51.224998, 0.000000]
   negate: 0
   occupied_thresh: 0.65
   free_thresh: 0.196
   ```

> [!NOTE]
> Only change `resolution` and `image`, as other metrics are related to the greyscale factor of the map. The value for `resolution` refers to meters per pixel. (E.g. `0.05` means each pixel is a 5 cm x 5 cm square in the simulator.)


## Replacing F1TENTH Gym ROS Maps with Racerbot A Maps

To replace the default F1TENTH Gym ROS maps with the maps provided by `racerbot_a`, run:

```bash
cd /racerbot_ws/src
rm -rf f1tenth_gym_ros/maps/*
cp -a racerbot_a/maps/. f1tenth_gym_ros/maps/
