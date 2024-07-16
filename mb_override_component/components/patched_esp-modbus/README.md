This component represents the patched component of esp-modbus located in "patched_esp-modbus" folder.

How it works:

The original component version selected by project idf_component.yml file will be dowloaded
by component manager but its sources will be overriden by sources from "patched_esp-modbus/freemodbus/" folder.
Only the files that exist in this folder will be overriden during compilation. All other sources of the original component
will be used unchanged. This technique allows to override the code of original component without changing
the official esp-modbus component files. The source files to be patched needs to be added into
the same place where they located in the original component.
