# ALFA installation guide

Currently supported installations: </p>

- To Install ALFA on a Linux-based Desktop system:
  - [x] Ubuntu 22.04.1 LTS with ROS 2 Humble Hawksbill ([**Setup guide**](docs/guides/desktop_install.md))
- To install ALFA on an Embedded platform:
  - [x] Xilinx Zynq UltraScale+ MPSoC ZCU104 (Petaliunx and Vivado) ([**Setup guide**](docs/guides/zcu104_install.md))

## Desktop Ubuntu 22.04.1 LTS with ROS 2 Humble Hawksbill

ALFA is built on top of the [Robot Operating System (ROS)](https://www.ros.org/) architecture to read sensor's data, communicate with other modules, and to perform point cloud processing. Such processing often resorts to [Point Cloud Library (PCL)](https://pointclouds.org/) software modules. 

### 1. ROS2 Installation [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html) 

Make sure you have a locale which supports UTF-8:

```sh
locale
```

Enable the Ubuntu Universe Repository:

```sh
sudo apt install software-properties-common
```

```sh
sudo add-apt-repository universe
```

Add the ROS 2 GPG key with apt:

```sh
sudo apt update && sudo apt install curl
```

```sh
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the repository to your sources list:

```sh
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update your apt repository caches after setting up the repositories. It is important that systemd and udev-related packages are updated before installing ROS 2:

```sh
sudo apt update
```

```sh
sudo apt upgrade
```

(Full) Desktop Install (ROS, RViz, Dev tools, and Libraries):

```sh
sudo apt install ros-humble-desktop
```

Set up the environment by sourcing the following file:

```sh
source /opt/ros/humble/setup.bash
```

You can permanently do this by adding the following into the ~/.bashrc file:

```sh
# Enable ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
 source /opt/ros/humble/setup.bash
fi
```

Install colcon. You will need it for building ROS2 packages:

```sh
sudo apt install python3-colcon-common-extensions
```

### 2. Point Cloud Library (PCL)

Installing the ROS2 full desktop version, also installs the PCL. However, if another ROS version present in you distribution, PCL may have to be installed separately. Make sure the PCL is installed:

```sh
sudo apt install libpcl-dev
```

### 3. ALFA framework

We recommend you to have a folder named 'ALFA' anywhere you like and keep inside all related software and tools. You need the git tool installed.

```sh
sudo apt install git
```

```sh
git clone -b main https://github.com/alfa-project/alfa-framework.git
```

### 4. Create a ROS Workspace
To use the Desktop version of ALFA, only a few more steps are required. First, create a workspace for working with ALFA packages without interfering with the existing default ROS2 workspace. To know more about creating a workspace check [ROS2 documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). This will include all ROS2-related software provided by ALFA.

Create a Ros2 workspace inside the ALFA directory:

```sh
mkdir -p ros2alfa_ws/src && cd ros2alfa_ws/src
```

Then, link the provided ALFA ROS2 packages to the new ROS2 workspace:

- **alfa_node**

```sh
ln -s ../../alfa-framework/node alfa_node
```

- **alfa_msg**

```sh
ln -s ../../alfa-framework/msgs alfa_msg
```

- **software alfa-extensions**

```sh
ln -s ../../alfa-framework/extensions/sw alfa_ext
```

- **alfa-monitor**

```sh
ln -s ../../alfa-framework/monitor/ alfa_monitor
```

### 5. Compile ALFA extensions

ALFA software extensions are pieces of software written in C/C++ with ROS and ALFA-node dependencies. Therefore, compile them for desktop usage requires *colcon* within the ros2 workspace. Make sure that the packages alfa_node, alfa_msg and at least one extension (in this case we included the dummy extension) are inside your src folder and then build the workspace with the following commands:

```sh
cd ALFA/ros2alfa_ws 
```

```sh
colcon build 
```

The following output should appear (if other packages are present they will also build and the output will include them):

```sh
Starting >>> alfa_msg
Finished <<< alfa_msg [0.61s]                     
Starting >>> alfa_node
Finished <<< alfa_node [0.18s]                
Starting >>> alfa_monitor
Starting >>> ext_distance_filter
Starting >>> ext_dummy
Finished <<< ext_distance_filter [0.15s]                                                
Finished <<< ext_dummy [0.15s]
Finished <<< alfa_monitor [0.32s]                    

Summary: 5 packages finished [1.36s]
```

Source the workspace environment to run the extensions:

```sh
source ./install/setup.bash 
```

**Note**: To run an Extension, check the [Extension user guide](extensions_user_guide.md).

## Xilinx Zynq UltraScale+ MPSoC ZCU104

The following steps show how to run ALFA on the [Xilinx Zynq UltraScale+ MPSoC ZCU104](https://www.xilinx.com/products/boards-and-kits/zcu104.html). It is required that the [Desktop version](https://github.com/alfa-project/alfa-platforms/Desktop/README.md) of ALFA is already installed and running with all repositories and source at their correct location. 

This guide was based on: 
- https://news.accelerationrobotics.com/ros2-humble-yocto-petalinux
- https://docs.xilinx.com/r/en-US/ug1144-petalinux-tools-reference-guide

### 1. Tools Installation and setup

**Note**: Downloading, installing, and building software packages with Xilinx's PetaLinux requires a lot of free space on your local hard drive. Reserve at least 100GB. 

Tested with the following tools:
- Ubuntu 22.04.4 LTS
- Xilinx Petalinux 2022.2 (that comes with Yocto Honister)
- ROS2 Foxy Fitzroy (https://github.com/ros/meta-ros.git and compatible with Honister)

Create a folder 'alfa-embedded' inside the ALFA folder previously created:

```sh
mkdir -p alfa-embedded/petalinux
```

Download the Petalinux installer from Xilinx website ([link](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/embedded-design-tools.htm)) (registration is required) to the folder 'alfa-embedded' and change the file properties to make it executable.

```sh
chmod 755 ./petalinux-v2022.2-10141622-installer.run
```
Run it (a different installation directory can be set with the -d option):
```sh
./petalinux-v2022.2-10141622-installer.run -d petalinux
```

**Note**: Install any other possible missing dependencies/libraries reported by the installer.

Go inside the petalinux folder and source its settings:

```sh
source settings.sh
```

### 2. Create a project from the ALFA hardware xsa file

To create a project for the zcu104, inside the ‘projects’ dir run petalinux-create command (***-n** defines the name of the project and -t the name of the base template*):

```sh
petalinux-create --type project --template zynqMP --name alfa_zcu104
```

```sh
cd alfa_zcu104
```

```sh
petalinux-config --get-hw-description=/alfa-framework/alfa-platforms/xilinx/hardware/zcu104.xsa
```

Change the Machine name from 'template' to 'zcu104-revc' under the DTG Settings menu and save and exit the config file. 
```sh
(zcu104-revc) MACHINE_NAME
```

Select the Root filesystem type under the Image Packaging Configuration menu
```sh
Root filesystem type (EXT4 (SD/eMMC/SATA/USB))
```

### 3. Add meta-layers for ROS2 Humble and configure them in PetaLinux 

Create a new folder named "layers" inside your project folder alfa_zcu104. Inside the layers folder, clone the meta-ros layers from github:

```sh
git clone -b honister https://github.com/ros/meta-ros.git 
```

Add the following into the alfa_zcu104/build/conf/bblayers.conf file in the BBLAYERS definition, for the meta-ros recipes. If there are already any ros lines, delete them and add the following:

```sh
${SDKBASEMETAPATH}/../../layers/meta-ros/meta-ros-common \
${SDKBASEMETAPATH}/../../layers/meta-ros/meta-ros2 \
${SDKBASEMETAPATH}/../../layers/meta-ros/meta-ros2-foxy \
```

Edit alfa_zcu104/components/yocto/layers/meta-petalinux/conf/distro/petalinux.conf to change the variable ROS_DISTRO to foxy:

```sh
ROS_DISTRO = "foxy"
```
<!---------------------->

### 4. Extend the minimal image to include ROS2 

Update petalinux image to include the new ROS2 content. Inside the project folder create a new dir:

```sh
mkdir -p project-spec/meta-user/recipes-image/images
```

Inside the 'images' dir create an empty file 'ros-petalinux.bb':

```sh
touch ros-petalinux.bb
```

Open the file and insert the following lines:
<details>
  <summary>Click to expand</summary>

```sh
require ${COREBASE}/../meta-petalinux/recipes-core/images/petalinux-image-minimal.bb

SUMMARY = "A small image just capable of starting ROS2."
DESCRIPTION = "${SUMMARY}"

inherit ros_distro_${ROS_DISTRO}
inherit ${ROS_DISTRO_TYPE}_image

ROS_SYSROOT_BUILD_DEPENDENCIES = " \
 ament-lint-auto \
 ament-cmake-auto \
 ament-cmake-core \
 ament-cmake-cppcheck \
 ament-cmake-cpplint \
 ament-cmake-export-definitions \
 ament-cmake-export-dependencies \
 ament-cmake-export-include-directories \
 ament-cmake-export-interfaces \
 ament-cmake-export-libraries \
 ament-cmake-export-link-flags \
 ament-cmake-export-targets \
 ament-cmake-gmock \
 ament-cmake-gtest \
 ament-cmake-include-directories \
 ament-cmake-libraries \
 ament-cmake \
 ament-cmake-pytest \
 ament-cmake-python \
 ament-cmake-ros \
 ament-cmake-target-dependencies \
 ament-cmake-test \
 ament-cmake-version \
 ament-cmake-uncrustify \
 ament-cmake-flake8 \
 ament-cmake-pep257 \
 ament-copyright \
 ament-cpplint \
 ament-flake8 \
 ament-index-python \
 ament-lint-cmake \
 ament-mypy \
 ament-package \
 ament-pclint \
 ament-pep257 \
 ament-pycodestyle \
 ament-pyflakes \
 ament-uncrustify \
 ament-xmllint \
 cmake \
 eigen3-cmake-module \
 fastcdr \
 fastrtps-cmake-module \
 fastrtps \
 git \
 gmock-vendor \
 gtest-vendor \
 pkgconfig \
 python-cmake-module \
 python3-catkin-pkg \
 python3-empy \
 python3 \
 python3-nose \
 python3-pytest \
 rcutils \
 rmw-implementation-cmake \
 rosidl-cmake \
 rosidl-default-generators \
 rosidl-generator-c \
 rosidl-generator-cpp \
 rosidl-generator-dds-idl \
 rosidl-generator-py \
 rosidl-parser \
 rosidl-runtime-c \
 rosidl-runtime-cpp \
 rosidl-typesupport-c \
 rosidl-typesupport-cpp \
 rosidl-typesupport-fastrtps-cpp \
 rosidl-typesupport-interface \
 rosidl-typesupport-introspection-c \
 rosidl-typesupport-introspection-cpp \
 foonathan-memory-vendor \
 libyaml-vendor \
"

IMAGE_INSTALL:append = " \
 ros-base \
 cyclonedds \
 rmw-cyclonedds-cpp \
 tmux \
 byobu \
 python3-argcomplete \
 glibc-utils \
 localedef \
 rt-tests \
 stress \
 xrt-dev \
 xrt \
 zocl \
 opencl-headers-dev \
 opencl-clhpp-dev \
 ${ROS_SYSROOT_BUILD_DEPENDENCIES} \
"

EXTRA_IMAGE_FEATURES += "ros-implicit-workspace"

```

</details>

Save and close the file.

### 5. Add ALFA components or other custom applications 

Open the file alfa_zcu104/build/conf/bblayers.conf and add the following line to the BBLAYERS definition to add the ALFA layer meta-alfa (which is located under your alfa-framework installation): 

```sh
${SDKBASEMETAPATH}/../../../../../alfa-framework/meta-alfa \
```

Replace with the following lines into the device tree user file located in alfa_zcu104/project-spec/meta-user/recipes-bsp/device-tree/files/system-user.dtsi

<details>
  <summary>Click to expand</summary>

```sh
/include/ "system-conf.dtsi"
/ {
 reserved-memory {
  #address-cells = <2>;
  #size-cells = <2>;
  ranges;
  
  reserved_cachable_alfa: reserved_cachable_alfa@5000000 {
   compatible = "shared-dma-pool";
   reg = <0x0 0x50000000 0x0 0x1000000>;
   cache-policy = "writeback";
   access = "read-write";
   no-map;
  };
  
  reserved_non_cachable_alfa: reserved_non_cachable_alfa@40000000 {
   compatible = "shared-dma-pool";
   reg = <0x0 0x40000000 0x0 0x1000000>;
   no-map;
  };
  };
    
  alfadd: alfadd{
   status = "okay";
   compatible = "alfadd";
   reg = <0x0 0x40000000 0x0 0x1000000>, <0x0 0x50000000 0x0 0x1000000>;
 };
  axi_cdma_0 {
  status = "disabled";
 };
 };
```

</details>

Update ros-petalinux.bb receipe inside the folder "alfa_zcu104/project-spec/meta-user/recipes-image/images" with the desired alfa components under 'IMAGE_INSTALL:append':

```sh
 alfa-dd \
 alfa-msg \
 alfa-node \
 ext-dummy \
 ext-distance-filter \
```

Add the following line at the end of the file to load the ALFA device driver during boot:

```sh
KERNEL_MODULE_AUTOLOAD = "alfadd"
```

Since ALFA extensions require root access to physical memory (it will be fixed soon), the final image must be configurated to enable it. Run the following command inside the project folder and select the *Petalinux RootFS Settings* menu to add a new user (alfa), and add this user to the sudo user.

```sh
petalinux-config -c rootfs
```
Add extra user alfa (no default password)
```sh
(root:root;alfa::passwd-expire;) Add Extra Users 
(alfa) Add Users to Sudo users
```

### 6. Build the image
Finally start petalinux-build to build our custom Linux image:

```sh
petalinux-build -c ros-petalinux
```

This should create a Linux image for zcu104 and display no errors. If some errors appear, the possibilities are: different package releases, corrupted downloaded files, low RAM memory, low disk space, low swap memory, or network-related issues. In case of memory issues, usually changing the number of parallel jobs to a lower number, and/or increasing the size of the swap memory solves the problem. 

### 7. Generate boot components and SDcard image:

The build images are located under the project folder alfa_zcu104/images/linux directory. A copy is also placed in the /tftpboot directory if the option is enabled in the system-level configuration for the PetaLinux project.

**Important:** By default, besides the kernel, RootFS, and U-Boot, the PetaLinux project is configured to generate and build the other boot components for Zynq FSBL, Zynq UltraScale+ MPSoC FSBL and PMU firmware, and for Versal PLM and PSM firmware. For more details on the auto generated boot components, see [UG1144 - Generating Boot Components](https://docs.xilinx.com/r/en-US/ug1144-petalinux-tools-reference-guide/Generating-Boot-Components).


A boot image usually contains a first stage boot loader image (Zynqmp_fsbl.elf), FPGA bitstream (project1.bit), PMU firmware (pmufw.elf), TF-A (bl31.elf), device tree description (system.dtb), and U-Boot (u-boot.elf).

Generate the boot image in .BIN format to include these components. This also includes the default bitstrem file. 

```sh
petalinux-package --boot --fpga --u-boot
```

Later, when a new FPGA bitstream is created with hardware extensions, it can be manually included in the boot.bin:

```sh
petalinux-package --boot --fpga <path-to-file>/system.bit --u-boot --force
```

Use the wic packaging tool to create a ready to use SD Card image with the following command:

```sh
petalinux-package --wic
```

**Note:** The default image generated by petalinux is around 6GB. This much more than we actually need. To change this, we can customize the wks file used by wic by changing the partition size accordingly. The default rootfs.wks file can be found inside the build directory. This file is autogenerated. Copy it to the 'projects' directory, change the size of the boot and filesystem partitions, and include it with the petalinux-package command:

```sh
petalinux-package --wic --wks <path-to-file>/rootfs.wks 
```

The rootfs can also be changed with petalinux (optional) to remove unwanted packages. This also helps in reducing the final image’s size:

```sh
petalinux-config -c rootfs
```

Flash the created petalinux-sdimage.wic image into the SD card. This file can be found inside the project folder/images/linux. Using dd command (change the bs parameter accordingly. This will highly increase the flashing process):

```sh
sudo dd if=petalinux-sdimage.wic of=/dev/sda conv=fsync bs=8M
```

Or you can use the pv command. This gives a fancy progress bar and selects the best block size parameter:

```sh
sudo bash -c 'pv *.wic > /dev/sda'
```

Finally, insert the sd card into the board and boot. Login with the root user and the password defined in the rootfs configuration.

### 8. Install more embedded software extensions

To include custom extensions in the embedded image after testing the framework, it is necessary a recipe file for each new extension. Existing recipes (.bb files) in the meta-alfa/recipes-alfa/alfa-extensions folder can be used as a starting point. 

#### Update the ros-petalinux.bb receipt 

Iinside folder "alfa_zcu104/project-spec/meta-user/recipes-image/images" add the new recipe under 'IMAGE_INSTALL:append':

```sh
 ext-<new_extension_name> \
```

#### Update the image

```sh
petalinux-build -c ros-petalinux
```

#### Updated the card image with the wic command and copy it to the SD card:

```sh
petalinux-package --wic
```
```sh
sudo dd if=images/linux/petalinux-sdimage.wic of=/dev/sda conv=fsync bs=8M
```

### 9. Check the created embedded [ALFA extensions](https://github.com/alfa-project/alfa-extensions/) with the Xilinx Zynq UltraScale+ MPSoC ZCU104 platform. 

The default username is "alfa". Pick a password at your choice.  
```sh
[  OK  ] Finished Permit User Sessions.
[  OK  ] Started Getty on tty1.
[  OK  ] Started Serial Getty on ttyPS0.
[  OK  ] Reached target Login Prompts.
[  OK  ] Started Target Communication Framework agent.
[  OK  ] Reached target Multi-User System.
         Starting Record Runlevel Change in UTMP...
[  OK  ] Finished Record Runlevel Change in UTMP.

PetaLinux 2022.2_release_S10071807 alfazcu104 ttyPS0

alfazcu104 login: alfa
Password: 
```

<b>NOTE:</b> From petalinux 2022.1 onwards, the root login is disabled by default. However, at this point of development, to communicate with the hardware the <b>alfadd</b> requires root access to the dev/mem device driver. The current work around is to enable super user acess with "sudo -i" after login.

Set an IP address in the same network as the host system (that will play the ros2 bags and the monitor tool) and set the default gateway.
```sh
sudo -i ifconfig eth0 192.168.1.2 255.255.255.0
sudo -i route add default gw 192.168.1.1
```
Ping the host system to check any connectivity issues and list the available topics.
```sh
ping 192.168.1.1
```
```sh
ros2 topic list
```

### 10.  Install embedded hardware extensions
Available Soon!

<!----
To use hardware extensions with the ZCU104 board, you need to [install Xilinx's Vivado](https://www.xilinx.com/support/download.html). At the time of this writing ALFA has been tested with Vivado Design Suite - HLx Editions - 2022.2. We recommend you to get familiarized with this tool suite before developing hardware Extensions. Useful documentation can be found here: [Vivado Design Suit User Guide UG910](https://docs.xilinx.com/r/en-US/ug910-vivado-getting-started).

### Create an ALFA project, add the extension and generate the bitstream

ALFA projects can be created using the TCL script that setups all components and the required configurations. To run this script, go to Xilinx/scripts/ folder, make the script *setup_vivado_project* executable if is not already and run it with the name of the project that you want and the board as argument (only zcu104 is supported at the moment):

```sh
cd PATH_TO_ALFA_PLATFORMS/Xilinx/scripts
```

```sh
chmod +x setup_vivado_project
```

```sh
./setup_vivado_project <project_name> <board>
```

The script will open the Vivado GUI and create a new project with the name provided as argument (the project will be created in the folder *PATH_TO_ALFA_FRAMEWORK/alfa-platforms/vivado_projects/<project_name>*). Then, it will add the ALFA unit and board SoC to the design. Add the extension to the block design and connect all the remaining unconnected interfaces.

**Note**: only the native extensions will appear automatically in the IP catalog. Your extensions IP must be included in this project IP catalog to make them available in block design.

Please adjust the number of Debug Points and User Define interfaces required for your extension. Finally, create a wrapper for the design and proceed to generate the bitstream.

### Enable ALFA hardware extensions in the ALFA-Node

The ALFA-Node has the capability to run hardware extensions when the extensions are compiled with EXT_HARDWARE compilation flag defined. This flag is defined in the CMakeLists.txt of the extensions and can be toggled on and off by commenting the line with a "#":

```cmake
20. add_compile_definitions(EXT_HARDWARE)
```

**Note:** This flag alters the node behavior to accommodate the hardware extensions. Therefore, developers when developing their extensions must define the extension behavior when this flag is defined. Check the dummy extension src files for an example of how to do this.

Update, if not already, the ros-petalinux.bb receipt inside folder "/project-spec/meta-user/recipes-image/images" with the desired ALFA extensions under 'IMAGE_INSTALL:append':

```sh
 ext-dummy \
```

Then go to your project folder and rerun the following command to update the image:

```sh
petalinux-build -c ros-petalinux
```

Add the generated bitstream into the final image with the command:

```sh
petalinux-package --boot --fpga <PATH_TO_YOUR_BITSTREAM>/<BITSTREAM_FILE_NAME>.bit --u-boot --force
```

Create the SD Card image with the wic command:

```sh
petalinux-package --wic
```

And finally, copy the image to the SD Card (check the SD Card device name before running the command):

```sh
sudo dd if=images/linux/petalinux-sdimage.wic of=/dev/sda conv=fsync bs=8M
```



#### 4. Package and create new ALFA hardware extensions

In order to create and maintain the best environment for your extensions, we advise the usage of the package IP tool provided by Vivado. By doing it so, the step to include your extension in the ALFA environment will be the same as the native extensions.

The following steps describe how to package ALFA hardware extensions, using dummy extension as an example. Those are meant to be done in Xilinx's Vivado, with a project already created for your board and with the extension source files already in your computer. Start including the ALFA hardware components to the IP catalog:

1. Enter into the menu Window ⇾ IP Catalog.

    ![ipcat](figures/menu_ipcat.png)

2. Right click on Vivado Repository ⇾ Add Repository...

    ![addrep](figures/add_rep.png)

3. Select the path to the alfa-unit repository (PATH_TO_ALFA_FRAMEWORK_REP/alfa-framework/alfa-unit). If the right path is selected, a pop-up will show that 1 IP and 5 interfaces were added to the project.

    ![addrep](figures/add_unit_rep.png)

4. The ALFA-Unit IP should appear on Block Design's IP list.

    ![addrep](figures/alfa_unit_rep.png)

Then, package the extension with your hardware files and the ALFA interfaces:

1. Go to the **Tools** menu and select the **Create and Package New IP...**

    ![createip](figures/tools_createip.png)

2. After pressing **Next** in the first menu, select the option "Package a specified directory" in the Package Options section. Then press **Next**.

3. Select the dummy extension directory (directory containing all the hardware files ⇾ PATH_TO_ALFA_FRAMEWORK/alfa-extensions/hw/ext_dummy/) and press **Next**.

4. Then, change the *Project name* field to "dummy_extension" and let the Project location be the default path for your Vivado projects.

5. Finishing the process by clicking *Finish*. A new Vivado window will pop-up with your dummy extension package on it.

6. In order to take full advantage of Vivado Block Design features, we need to identify the ALFA interfaces present in the dummy. Select the menu *Port and Interfaces* of the Package IP and select the menu *Auto infer interface*:

    ![auto infer](figures/auto_infer.png)

7. Select User⇾Cartesian_representation_rtl and press *OK*. (**Note**: if none of ALFA interfaces are shown in the menu, the ALFA interfaces are not present in the IP catalog. For more information, check the [ALFA-Unit Integration section](https://github.com/alfa-project/alfa-unit#integration))

    ![cartesian](figures/cartesian_represent.png)

8. Repeat the process for the User⇾Extension_Interface_rtl.

9. The *Ports and interfaces* menu should look like this:

    ![ports](figures/ports_inter.png)

10. Associate the clock signal for both interfaces by right-clicking on top of them and selecting associate clock. Then select the *i_SYSTEM_clk* checkbox and press *OK*.
  
    ![associate clock](figures/associate_clock.png)  

11. Select the *Review and Package* submenu and press the *Package IP* button to finalize the process of packaging the ALFA extension:

    ![package](figures/package_ip.png)

To use the generated IP, you can follow the [steps above](#create-an-alfa-project-add-the-extension-and-generate-the-bitstream) to include the extension in the ALFA project and generate the bitstream. Then, **Jump to [ALFA extensions](https://github.com/alfa-project/alfa-extensions) to see how to run and interact with dummy node after booting your board.**

---->
