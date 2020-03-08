FROM arm32v7/python:3.8.2-buster


# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive


COPY requirements.txt .


RUN apt-get update -q \
    && apt-get install -qy --no-install-recommends \
        apt-transport-https \
        apt-utils \
        build-essential \
        curl \
        cmake \
        git \
        software-properties-common \
        unzip \
        python3-dev \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        python3-yaml \
        imagemagick \
        gphoto2 \
    && pip3 install --upgrade pip \
    #&& pip3 install -r requirements.txt \
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=
