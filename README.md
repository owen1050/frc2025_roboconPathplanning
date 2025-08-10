This repo has 3 things:

1. An april tag JSON which can be imported instead of the 2025 welded field to update the location of the tags at robocon

2. An updated field image for path planner and advantage scope so you can see the robocon field in those programs
   - In advantage scope odometry (2d) field widget select help, then show assets folder. copy "Field2d_robocon" from this repo to your user assets folder. Restart advantage scope then it will be an option under the bottom right game field
   - In the app settings of pathplanner select field image, import custom. Chose image.jpg from this repo in the userassets/field2d folder and use a pixels/meter of 80.64

4. An example 2025 robot project which shows you how to update path planner lib with the smaller field and how to update your april tag json. 

If you have any questions feel free to ask in the CD thread:https://www.chiefdelphi.com/t/how-to-adapt-pathplanner-to-off-season-field-dimensions/505069/6?u=owen_busler
