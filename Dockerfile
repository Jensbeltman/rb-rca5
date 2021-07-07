FROM gazebo:gzserver9

WORKDIR /work

COPY . /work

CMD ["bash", "gazebo_server.sh", "bigworld.world"]
