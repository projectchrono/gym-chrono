FROM ubuntu:22.04 

ENV DISPLAY=:1 \
    VNC_PORT=5901 \
    NO_VNC_PORT=6901 \
    HOME=/sbel \
    TERM=xterm \
    STARTUPDIR=/dockerstartup \
    NO_VNC_HOME=/sbel/noVNC \
    DEBIAN_FRONTEND=noninteractive \
    VNC_COL_DEPTH=24 \
    VNC_RESOLUTION=1600x900 \
    VNC_PW=sbel
EXPOSE $VNC_PORT $NO_VNC_PORT

####################################################
# Install TigerVNC, noVNC, XFCE
####################################################
RUN export LIB_DIR="lib" && export IOMP5_DIR="" \
    && apt-get update && apt-get -y install unzip wget python3 python3-pip \
    git cmake ninja-build doxygen libvulkan-dev pkg-config libirrlicht-dev \
    freeglut3-dev mpich libasio-dev libboost-dev libglfw3-dev libglm-dev \
    libglew-dev libtinyxml2-dev swig python3-dev libhdf5-dev libnvidia-gl-515 libxxf86vm-dev \
    && ldconfig && apt-get autoclean -y && apt-get autoremove -y
RUN apt-get update && apt-get install -y net-tools bzip2 procps python3-numpy
    # TigerVNC
RUN apt-get update && apt-get install -y tigervnc-standalone-server \
    && printf '\n# sbel-docker:\n$localhost = "no";\n1;\n' >>/etc/tigervnc/vncserver-config-defaults \
    # noVNC
    && mkdir -p $NO_VNC_HOME/utils/websockify \
    && wget -qO- https://github.com/novnc/noVNC/archive/refs/tags/v1.3.0.tar.gz | tar xz --strip 1 -C $NO_VNC_HOME \
    && wget -qO- https://github.com/novnc/websockify/archive/refs/tags/v0.10.0.tar.gz | tar xz --strip 1 -C $NO_VNC_HOME/utils/websockify \ 
    && ln -s $NO_VNC_HOME/vnc_lite.html $NO_VNC_HOME/index.html \
    # XFCE
    && apt-get install -y supervisor xfce4 xfce4-terminal xterm dbus-x11 libdbus-glib-1-2 \
    && apt-get autoclean -y && apt-get autoremove -y \
    # Ensure $STARTUPDIR exists
    && mkdir $STARTUPDIR

####################################################
# Add scripts
####################################################
ADD ./src/ $HOME/src/
ADD ./desktop/ $HOME/Desktop/
RUN mv $HOME/src/.bashrc $HOME/.bashrc && chmod a+x $HOME/.bashrc $HOME/src/vnc_startup.sh $HOME/src/wm_startup.sh $HOME/src/buildChrono.sh $HOME/src/gymchrono.sh
WORKDIR /sbel

####################################################
# OptiX
####################################################
ADD ./opitx/ $HOME/optix/
RUN chmod +x $HOME/optix/NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh && \
    mkdir /opt/optix && \
    $HOME/optix/NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh --prefix=/opt/optix --skip-license

# ####################################################
# # Build Chrono
# ####################################################
RUN apt-get update \
    && apt-get install -y libthrust-dev libeigen3-dev libxinerama-dev libxcursor-dev \
    libxi-dev \
    && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin && mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
    && wget https://developer.download.nvidia.com/compute/cuda/12.1.0/local_installers/cuda-repo-ubuntu2204-12-1-local_12.1.0-530.30.02-1_amd64.deb \
    && dpkg -i cuda-repo-ubuntu2204-12-1-local_12.1.0-530.30.02-1_amd64.deb \
    && cp /var/cuda-repo-ubuntu2204-12-1-local/cuda-*-keyring.gpg /usr/share/keyrings/ && apt-get update && apt-get -y install cuda && rm $HOME/cuda-repo-ubuntu2204-12-1-local_12.1.0-530.30.02-1_amd64.deb
RUN $HOME/src/buildChrono.sh
####################################################
# Setup gymchrono
####################################################
RUN pip install --upgrade pip && pip install gymnasium stable_baselines3[extra]
# # RUN pip install gdown
RUN $HOME/src/gymchrono.sh
    
    
    
    # wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    # && dpkg -i cuda-keyring_1.1-1_all.deb && apt-get update \
    # && apt-get -y install cuda-toolkit-12-3 && rm \
    # $HOME/cuda-keyring_1.1-1_all.deb
# run the buildchrono script

#####################################################
# Startup
#####################################################
ENTRYPOINT ["/sbel/src/vnc_startup.sh"]