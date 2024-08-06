# MLIR-TOY
    
This project compiles chapter 2 of the LLVM MLIR tutorial https://mlir.llvm.org/docs/Tutorials/Toy/Ch-2/.

The launch parameters are defined in the vscode launch.json file: 

 
                "version": "0.2.0",
                    "configurations": [
                    {
                        "type": "lldb",
                        "request": "launch",
                        "name": "Launch",
                        "program": "${command:cmake.launchTargetPath}",
                        "args": ["<path>/mlir-toy/ast.toy", "-emit=mlir", "-mlir-print-debuginfo"],
                        "cwd": "${command:cmake.launchTargetDirectory}"
                    } 
                    ] 

hope this helps...

#Chapter 5 : Partial Lowering to Lower-Level Dialects for Optimization,
is in branch ch5

#Chapter 6 : Lowering to LLVM and CodeGeneration
is in branch ch6 
