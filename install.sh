# Download the SDK from: 
# Extract SDK
# Copy lib files to install locations
sudo cp -r FSLP_Files/ /usr/local/lib/

# Do Build.sh is modified to look like:
gcc -v -c -fpic -o Client_Dispatcher_32.o Client_Dispatcher.c -I.
gcc -v -c -fpic -o Client_API_32.o Client_API.c -I.
gcc -v -c -fpic -o Client_Packager_32.o Client_Packager.c -I.
gcc -v -c -fpic -o Serializer_BuiltIn_32.o Serializer_BuiltIn.c -I.
gcc -v -c -fpic -o Serializer_Struct_32.o Serializer_Struct.c -I.
gcc -v -c -fpic -o UART_Connector_32.o UART_Connector.c -I.
#gcc -v -shared -o C_SDK_32.dll Client_API_32.o Client_Dispatcher_32.o Client_Packager_32.o Serializer_BuiltIn_32.o Serializer_Struct_32.o UART_Connector_32.o UART_HalfDuplex64.def -I.
gcc -v -fpic -shared -o libboson.so Client_API_32.o Client_Dispatcher_32.o Client_Packager_32.o Serializer_BuiltIn_32.o Serializer_Struct_32.o UART_Connector_32.o -I.

#or Run
# sh do_build.sh
sudo cp libboson.so /usr/local/lib/
 
sudo chmod 644 /usr/local/lib/libboson.so 

sudo mkdir /usr/local/include/boson
sudo cp *.h /usr/local/include/


nm -A /usr/local/lib/libboson.so | grep Init
nm -A /usr/local/lib/libboson.so | grep Init
readelf --dyn-syms /usr/local/lib/libboson.so | grep Init

sudo cp ~/Projects/ConvergenceCamera/convergence_camera/BosonSDK/ClientFiles_C/libboson.so /usr/local/lib/

sudo cp ~/Projects/ConvergenceCamera/convergence_camera/BosonSDK/ClientFiles_C/libboson.so /usr/local/lib/

