FROM klee/klee:2.3
#FROM klee/klee:2.2
USER root
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 42D5A192B819C5DA
RUN apt-get update
RUN apt-get install software-properties-common -y
RUN add-apt-repository -y ppa:openjdk-r/ppa
RUN apt-get update
RUN apt-get install openjdk-8-jdk -y
#RUN update-alternatives --config java
#RUN update-alternatives --config javac
RUN apt-get update
RUN apt-get install apt-transport-https curl gnupg -yqq
RUN echo "deb https://repo.scala-sbt.org/scalasbt/debian all main" | tee /etc/apt/sources.list.d/sbt.list
RUN echo "deb https://repo.scala-sbt.org/scalasbt/debian /" | tee /etc/apt/sources.list.d/sbt_old.list
RUN curl -sL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x2EE0EA64E40A89B84B2DF73499E82A75642AC823" | gpg --no-default-keyring --keyring gnupg-ring:/etc/apt/trusted.gpg.d/scalasbt-release.gpg --import
RUN chmod 644 /etc/apt/trusted.gpg.d/scalasbt-release.gpg
RUN apt-get update
RUN apt-get install sbt -y
ADD https://github.com/Kitware/CMake/releases/download/v3.27.7/cmake-3.27.7-Linux-x86_64.sh /tmp/cmake-install.sh
RUN chmod u+x /tmp/cmake-install.sh 
RUN mkdir /opt/cmake-3.27.7 
RUN /tmp/cmake-install.sh --skip-license --prefix=/opt/cmake-3.27.7 
RUN rm /tmp/cmake-install.sh 
RUN ln -s /opt/cmake-3.27.7/bin/* /usr/local/bin
RUN apt-get update

#RUN echo "deb https://dl.bintray.com/sbt/debian /" | tee -a /etc/apt/sources.list.d/sbt.list
#RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 2EE0EA64E40A89B84B2DF73499E82A75642AC823
#RUN apt-get update
#RUN apt-get install sbt
RUN apt-get install git make autoconf flex bison less nano libboost-all-dev clang-tidy -y
ENV PATH="$PATH:/tmp/llvm-110-install_O_D_A/bin"
#ENV BASE=/tmp
#RUN update-alternatives --install /usr/bin/c++ c++ $(command -v clang++) 1000
#RUN update-alternatives --install /usr/bin/cc  cc  $(command -v clang)   1000
USER klee
#ENV CC="clang"
#ENV CXX="clang++"
WORKDIR /home/klee
RUN git clone http://git.veripool.org/git/verilator
WORKDIR /home/klee/verilator
RUN git pull
RUN git checkout v4.222
RUN autoconf
RUN ./configure
RUN make -j8
USER root
RUN make install
COPY src/ignore.clang-tidy /usr/local/share/verilator/include/.clang-tidy
#ADD verilated.mk /usr/local/share/verilator/include/verilated.mk

#ENV LLVM_COMPILER="clang"
#RUN apt-get install python3 python3-pip valgrind -y
#USER klee
#ENV PATH="$PATH:/home/klee/.local/bin/"
#RUN pip3 install wllvm
#ENV CC="wllvm"
#ENV CXX="wllvm++"
ADD microrv32 /home/klee/microrv32
RUN chown klee:klee /home/klee/microrv32 -R
WORKDIR /home/klee/microrv32
RUN make clean
RUN make microrv32/rtl/RiscV32Core.v
#RUN make
#RUN cp sim_main.cpp obj_dir/sim_main.cpp
#WORKDIR /home/klee/microrv32/obj_dir
USER klee
WORKDIR /home/klee/
CMD ["/bin/bash"]
