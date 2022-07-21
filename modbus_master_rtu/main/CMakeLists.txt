idf_component_register(
                    SRCS "main.c" "file_server.c" "master.c" "modbus_params.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "favicon.ico" "upload_script.html")
