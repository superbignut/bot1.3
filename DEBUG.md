#### debugging record

+ if I want to use build-Project-bottom in the state line to build the project,

    it seems that .vscode/c_cpp_properties.json is necessarry

    such as bolows:

    ``` json
        {
            "configurations": [
                {
                    "name": "ESP-IDF",
                    "compilerPath": "${config:idf.toolsPath}/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc",
                    "compileCommands": "${config:idf.buildPath}/compile_commands.json",
                    "includePath": [
                        "${config:idf.espIdfPath}/components/**",
                        "${config:idf.espIdfPathWin}/components/**",
                        "${workspaceFolder}/**"
                    ],
                    "browse": {
                        "path": [
                            "${config:idf.espIdfPath}/components",
                            "${config:idf.espIdfPathWin}/components",
                            "${workspaceFolder}"
                        ],
                        "limitSymbolsToIncludedHeaders": true
                    }
                }
            ],
            "version": 4
        }

    ```

    Instead, Use Makefile is more comfortable...


+ I don't know why include follows into KEY.h will cause error when build, but moving to KEY.c is no error.
    
```cpp
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
```