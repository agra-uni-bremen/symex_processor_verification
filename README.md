# Symbolic Processor Verification

This verification approach implementation is based on: 

* [KLEE][klee github]
* [RISC-V VP][riscv-vp github]
* [µrv32---microrv32][microrv32 github]

## Quick Guide

## Clone Repo
```console
foo@bar:~$ git clone git@gitlab.informatik.uni-bremen.de:processor_verification/symex_processor_verification.git --recursive
```

### Build Docker Container
```console
foo@bar:~$ make build
```

### Run Docker Container
```console
foo@bar:~$ make run
```

### Build Bytecode / LLVM IR in Docker Container (Optional)
```console
klee@klee:~$ cd src && make bytecode
```

### Run KLEE in Docker Container
```console
klee@klee:~$ cd src && make
```

## How to Cite

Further details are described in the folliwng publication: [publication][symex-vp paper]:

```
@INPROCEEDINGS{nbruns2023symex,
  author={Bruns, Niklas and Herdt, Vladimir and Drechsler, Rolf},
  booktitle={2023 Design, Automation & Test in Europe Conference & Exhibition (DATE)}, 
  title={Processor Verification using Symbolic Execution: A RISC-V Case-Study}, 
  year={2023},
  volume={},
  number={},
  pages={1-6},
  doi={10.23919/DATE56975.2023.10137202}
  }
```

[klee github]: https://github.com/klee/klee	
[riscv-vp github]: https://github.com/agra-uni-bremen/riscv-vp
[microrv32 github]: https://github.com/agra-uni-bremen/microrv32
[symex-vp paper]: https://doi.org/10.23919/DATE56975.2023.10137202

