using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using System.IO.Ports;

using LibUsbDotNet;
using LibUsbDotNet.Main;

namespace CNCMicaMini
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        public static UsbDevice myUsbDevice;
        public static UsbDeviceFinder myUsbFinder = new UsbDeviceFinder(1155, 22370);
        UsbEndpointReader reader;
        UsbEndpointWriter writer;

        private void btn_Connect_Click(object sender, EventArgs e)
        {
            if (btn_Connect.Text == "Connect") //kết nối
            {
                try
                {
                    myUsbDevice = UsbDevice.OpenUsbDevice(myUsbFinder);
                    if (myUsbDevice == null) throw new Exception("Device Not Found.");
                    IUsbDevice wholeUsbDevice = myUsbDevice as IUsbDevice;
                    if (!ReferenceEquals(wholeUsbDevice, null))
                    {
                        wholeUsbDevice.SetConfiguration(1);
                        wholeUsbDevice.ClaimInterface(0);
                    }
                    reader = myUsbDevice.OpenEndpointReader(ReadEndpointID.Ep01);
                    writer = myUsbDevice.OpenEndpointWriter(WriteEndpointID.Ep01);
                    reader.DataReceived += (OnRxEndPointData);
                    reader.DataReceivedEnabled = true;
                    btn_Connect.Text = "Disconnect";
                }
                catch
                {
                    MessageBox.Show("error");
                }
            }
            else // ngắt kết nối
            {
                reader.DataReceivedEnabled = false;
                reader.DataReceived -= (OnRxEndPointData);
                reader.Dispose();
                writer.Dispose();
                if (myUsbDevice != null)
                {
                    if (myUsbDevice.IsOpen)
                    {
                        IUsbDevice wholeUsbDevice = myUsbDevice as IUsbDevice;
                        if (!ReferenceEquals(wholeUsbDevice, null))
                        {
                            wholeUsbDevice.ReleaseInterface(0);
                        }
                        myUsbDevice.Close();

                    }
                    myUsbDevice = null;
                    UsbDevice.Exit();
                }
                btn_Connect.Text = "Connect";
            }
        }

        private void OnRxEndPointData(object sender, EndpointDataEventArgs e)
        {
            Action<string> Action = addToTextBox;
            this.BeginInvoke(Action, (Encoding.Default.GetString(e.Buffer, 0, e.Count)));
        }
        private void addToTextBox(string input)
        {
            txt_Test.Text += input.Substring(1);
        }

        private void btn_Send_Click(object sender, EventArgs e)
        {
            /*if (textBox1.Text.Length > 63)
            {
                MessageBox.Show("Gửi không quá 63 kí tự");
            }*/
            try
            {
                int bytesWritten;
                writer.Write(Encoding.Default.GetBytes("\x02" + "hello"), 1000, out bytesWritten);
            }
            catch (Exception err)
            {
                MessageBox.Show("Can Not Send Data To USB Device\nDetails: " + err);
            }
        }

        /*
                //---------------fileds-----------------
                private int numOfCom = 0;
                private string rxbuff = "";
                private string txbuff = "";

                //--------------methods----------------------
                private void processrxbuffer(object sender, EventArgs e)
                {
                    txt_Test.Text = rxbuff;
                }

                //-----------------events-------------------------------------

                private void timer1_Tick(object sender, EventArgs e)
                {
                    string[] port = SerialPort.GetPortNames();
                    if( port.Length != numOfCom)
                    {
                        numOfCom = port.Length;
                        cbo_Select_Com.Items.Clear();
                        for( int i = 0; i < port.Length; i++ )
                        {
                            cbo_Select_Com.Items.Add(port[i]);
                        }
                    }

                }


                private void OnCom(object sender, SerialDataReceivedEventArgs e)
                {
                    rxbuff = COM.ReadLine();
                    this.Invoke(new EventHandler(processrxbuffer));
                }

                private void btn_Send_Click(object sender, EventArgs e)
                {
                    txbuff = "Hello, How are you?";
                    COM.Write(txbuff);
                }

                private void btn_Connect_Click(object sender, EventArgs e)
                {
                    COM.PortName = cbo_Select_Com.Text;
                    COM.Open();
                }*/
    }
}
