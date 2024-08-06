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

    
