#This script merges all the sdf_assets into the final model.sdf
rm -f model.sdf
cat sdf_assets/Body.xml sdf_assets/Suspension.xml sdf_assets/Steering.xml sdf_assets/Wheels.xml sdf_assets/Ender.xml > model.sdf
echo model.sdf built!

