# # Use the official Ubuntu 22.04 image as the base
# FROM ubuntu:22.04

# # Set environment variables to avoid prompts during installation
# ENV DEBIAN_FRONTEND=noninteractive
# ENV TZ=Etc/UTC
# ENV ROS_DISTRO=humble
# ENV LANG=en_US.UTF-8
# ENV LC_ALL=en_US.UTF-8

# # Install necessary locales and basic tools
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     locales \
#     sudo \
#     software-properties-common \
#     curl \
#     git \
#     wget \
#     build-essential \
#     cmake \
#     python3-pip \
#     libssl-dev \
#     libusb-1.0-0-dev \
#     libudev-dev \
#     pkg-config \
#     libgtk-3-dev \
#     libglfw3-dev \
#     libgl1-mesa-dev \
#     libglu1-mesa-dev \
#     v4l-utils \
#     unzip \
#     tzdata \
#     sshpass \
#     abootimg \
#     nfs-kernel-server \
#     qemu-user-static \
#     udev \
#     lsb-release \
#     gnupg2 \
#     && echo "$TZ" > /etc/timezone && \
#     ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
#     dpkg-reconfigure -f noninteractive tzdata \
#     && locale-gen en_US en_US.UTF-8 \
#     && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
#     && apt-get clean \
#     && rm -rf /var/lib/apt/lists/*






# # ROS2 Humble Installation
# RUN sudo apt update && sudo apt install locales -y && \
#     sudo locale-gen en_US en_US.UTF-8 && \
#     sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
#     sudo apt install -y software-properties-common && \
#     sudo add-apt-repository universe && \
#     sudo apt update && sudo apt install curl -y && \
#     sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
#     sudo apt update && sudo apt install -y \
#     python3-flake8-docstrings \
#     python3-pip \
#     python3-pytest-cov \
#     ros-dev-tools \
#     python3-flake8-blind-except \
#     python3-flake8-builtins \
#     python3-flake8-class-newline \
#     python3-flake8-comprehensions \
#     python3-flake8-deprecated \
#     python3-flake8-import-order \
#     python3-flake8-quotes \
#     python3-pytest-repeat \
#     python3-pytest-rerunfailures

# # Initialize and update rosdep
# RUN sudo rosdep init && rosdep update

# # Setup ROS2 workspace
# RUN mkdir -p ~/ros2_humble/src && cd ~/ros2_humble && \
#     vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src && \
#     rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" && \
#     cd ~/ros2_humble && colcon build --symlink-install

# # Install RealSense SDK from source (version 2.53.1)
# RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.53.1.zip && \
#     unzip v2.53.1.zip && \
#     cd librealsense-2.53.1 && \
#     echo "" | sudo ./scripts/setup_udev_rules.sh && \
#     sudo apt install -y v4l-utils && \
#     ./scripts/patch-realsense-ubuntu-lts-hwe.sh && \
#     mkdir build && cd build && \
#     cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true && \
#     sudo make uninstall && make clean && make -j$(nproc) && sudo make install

# # Set up ROS2 workspace for RealSense ROS wrapper
# RUN mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && \
#     git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master && \
#     cd ~/ros2_ws && \
#     sudo apt-get install python3-rosdep -y && \
#     sudo rosdep init && rosdep update && \
#     rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y && \
#     colcon build

# # Install ORB-SLAM3, GTSAM, OPENCV, etc.
# RUN git clone https://github.com/opencv/opencv.git && \
#     cd opencv && git checkout 4.2.0 && \
#     sudo apt-get update && sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev && \
#     sed -i 's/ipcp-unit-growth/ipa-cp-unit-growth/g' ./3rdparty/carotene/CMakeLists.txt && \
#     sed -i 's/ipcp-unit-growth/ipa-cp-unit-growth/g' ./3rdparty/carotene/hal/CMakeLists.txt && \
#     mkdir build && cd build && \
#     cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
#     sudo make install

# # Install Pangolin
# RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
#     cd Pangolin && mkdir build && cd build && \
#     cmake .. && cmake --build .

# # Install ORB-SLAM3
# RUN git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3 && \
#     cd ORB_SLAM3 && chmod +x build.sh && ./build.sh && \
#     sed -i '/bool System::isFinished()/a Frame System::GetCurrentFrame() { unique_lock<mutex> lock(mMutexState); return mpTracker->mCurrentFrame; }' /ORB_SLAM3/src/System.cc && \
#     sed -i '/void ChangeDataset();/a Frame GetCurrentFrame();' /ORB_SLAM3/include/System.h && \
#     chmod +x build.sh && ./build.sh

# # Set the entrypoint for ROS2
# ENTRYPOINT ["/bin/bash"]
# CMD ["-c", "source /opt/ros/$ROS_DISTRO/setup.bash && exec bash"]

# docker build -t my-arm64-ubuntu:22.04 .

FROM --platform=linux/arm64 ubuntu:22.04

# Set environment variables to avoid prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install necessary locales and basic tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales sudo software-properties-common curl git wget build-essential \
    cmake python3-pip libssl-dev libusb-1.0-0-dev libudev-dev pkg-config \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev v4l-utils \
    unzip tzdata sshpass abootimg nfs-kernel-server qemu-user-static udev \
    lsb-release gnupg2 libavcodec-dev libavformat-dev libswscale-dev \
    && echo "$TZ" > /etc/timezone && ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# # Install OpenCV 4.2.0
RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && git checkout 4.2.0 && \
    sed -i 's/ipcp-unit-growth/ipa-cp-unit-growth/g' ./3rdparty/carotene/CMakeLists.txt && \
    sed -i 's/ipcp-unit-growth/ipa-cp-unit-growth/g' ./3rdparty/carotene/hal/CMakeLists.txt && \
    sed -i '1i #include <thread>' modules/gapi/test/gapi_async_test.cpp && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && sudo make install

# # ROS2 Humble Installation
RUN sudo apt update && sudo apt install locales -y && \
    sudo locale-gen en_US en_US.UTF-8 && \
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    sudo apt install -y software-properties-common && \
    sudo add-apt-repository universe && \
    sudo apt update && sudo apt install curl -y && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    sudo apt update && sudo apt install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures

# # Initialize and update rosdep
RUN sudo rosdep init && rosdep update

# # Setup ROS2 workspace
RUN mkdir -p ~/ros2_humble/src && cd ~/ros2_humble && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" && \
    cd ~/ros2_humble && colcon build --symlink-install

# Install RealSense SDK from source (version 2.53.1)
RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.53.1.zip && \
    unzip v2.53.1.zip && cd librealsense-2.53.1 && \
    sed -i 's/udevadm control --reload-rules && udevadm trigger/echo "Skipping udev reload in Docker"/g' ./scripts/setup_udev_rules.sh && \
    sudo ./scripts/setup_udev_rules.sh && sudo apt install -y v4l-utils && \
    ./scripts/patch-realsense-ubuntu-lts-hwe.sh || true && \
    mkdir build && cd build && cmake ../ -DFORCE_RSUSB_BACKEND=true \
    -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true && \
    sudo make uninstall && make clean && make -j$(nproc) && sudo make install
    
# Set up ROS2 workspace for RealSense ROS wrapper
RUN mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master && \
    cd ~/ros2_ws && \
    sudo apt-get install python3-rosdep -y && \
    sudo rosdep init && rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y && \
    colcon build

# Install ORB-SLAM3, GTSAM, OPENCV, etc.
RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && git checkout 4.2.0 && \
    sudo apt-get update && sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev && \
    sed -i 's/ipcp-unit-growth/ipa-cp-unit-growth/g' ./3rdparty/carotene/CMakeLists.txt && \
    sed -i 's/ipcp-unit-growth/ipa-cp-unit-growth/g' ./3rdparty/carotene/hal/CMakeLists.txt && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    sudo make install

# Install Pangolin
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && mkdir build && cd build && \
    cmake .. && cmake --build .

# Install ORB-SLAM3
RUN git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3 && \
    cd ORB_SLAM3 && chmod +x build.sh && ./build.sh && \
    sed -i '/bool System::isFinished()/a Frame System::GetCurrentFrame() { unique_lock<mutex> lock(mMutexState); return mpTracker->mCurrentFrame; }' /ORB_SLAM3/src/System.cc && \
    sed -i '/void ChangeDataset();/a Frame GetCurrentFrame();' /ORB_SLAM3/include/System.h && \
    chmod +x build.sh && ./build.sh

# Set the entrypoint for ROS2
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "source /opt/ros/$ROS_DISTRO/setup.bash && exec bash"]
