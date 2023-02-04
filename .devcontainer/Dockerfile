ARG BASE_IMAGE
FROM $BASE_IMAGE

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    curl \
    git \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv

FROM arm64v8/ubuntu:20.04
ARG PYTHON_PACKAGES="python3 python3-dev python3-pip python3-venv"

RUN apt-get update && apt-get install --no-install-recommends -yq software-properties-common \
     && apt-get install -yq --no-install-recommends ${PYTHON_PACKAGES}
