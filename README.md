# intelligent-spectrum-sharing

Framework for intelligent and collaborative spectrum sharing experiments.

Further information can be found at the following papers:

[1] Felipe A. P. de Figueiredo, X. Jiao, W. Liu, R. Mennes, I. Jabandzić and I. Moerman, "A Spectrum Sharing Framework for Intelligent Next Generation Wireless Networks," in IEEE Access, vol. 6, pp. 60704-60735, 2018, doi: 10.1109/ACCESS.2018.2875047. [online]. Available: https://ieeexplore.ieee.org/document/8486941

[2] Felipe A. P. de Figueiredo, Xianjun Jiao, Wei Liu, Irfan Jabandzic, Spilios Giannoulis, and Ingrid Moerman, "A Framework for Intelligent Spectrum Sharing", 4th IEEE International Forum on Research and Technology for Society and Industry (RTSI) (RTSI 2018). [online]. Available: https://www.preprints.org/manuscript/201808.0447/v1



In order to compile the example and link it there are some steps that need to be followed.  
All of the below steps are tested on Ubuntu 14.04.5 LTS 64 bit.  

1) Install google protobufs library:   
We need to build it from source, as the native Ubuntu libprotobuf-dev library (that you can install through apt-get) is old  

    #setup dependencies  
    sudo apt-get install autoconf automake libtool curl make g++ unzip pkg-config

    #get source code  
    git clone https://github.com/google/protobuf.git  

    cd protobuf  
    ./autogen.sh  
    ./configure  
    make  
    make check  
    #If no errors are present  
    sudo make install  
    #refresh shared library cache.  
    sudo ldconfig   

2) Install zmq devel library  

    sudo apt-get install libzmq3-dev

OR if you want the latest version follow the instruction at:   
http://zeromq.org/intro:get-the-software (Alternative)  

 3) Now, follow these steps
 
 git clone https://github.com/zz4fap/intelligent-spectrum-sharing/

cd intelligent-spectrum-sharing

mkdir build

cd build

../compileProto.sh

cmake ../

sudo make


