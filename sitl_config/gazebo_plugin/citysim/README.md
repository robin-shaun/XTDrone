# Building

    mkdir build
    cd build
    cmake ..
    make install

## Updating world file

    cd worlds
    erb simple_city.world.erb > simple_city.world


# Running

1. Source setup file

    . [install_prefix]/share/citysim/setup.sh

1. Open world

    gazebo worlds/simple_city.world


