echo Generate DFU zip package
nrfutil pkg generate --application .\Output\Release\Exe\iguana_smart_module.hex --application-version 3 --application-version-string "3.0.0" --hw-version 52 --sd-req 0xB8 --key-file dfu_private_key.pem smartSack_v3_0.zip