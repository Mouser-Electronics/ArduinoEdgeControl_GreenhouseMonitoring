#include <Arduino_EdgeControl.h>
#include <openmvrpc.h>

openmv::rpc_scratch_buffer<256> scratch_buffer; // All RPC objects share this buffer.

openmv::rpc_software_serial_uart_master interface(2, 3, 19200);



void serial_print_example()
{
    String str = "Hello World @";
    str += millis();

    char buffer[str.length() + 1] {};
    str.toCharArray(buffer, sizeof(buffer));

    rpc.call("serial_print", buffer, sizeof(buffer));
}



void setup()
{
    EdgeControl.begin();
    
    Power.on(PWR_3V3);
    Power.on(PWR_VBAT);

    Power.on(PWR_MKR2);
    delay(5000); // Wait for MKR2 to power-on

    Serial.begin(115200);
    rpc.begin();
}




void loop()
{
    serial_print_example();
}
