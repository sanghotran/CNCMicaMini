using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Forms;
using System.Windows;

using System.IO.Ports;

using LibUsbDotNet;
using LibUsbDotNet.Main;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.IO;

namespace CNCMicaMiniWPF
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChangedEventHandler handler = PropertyChanged;
            if (handler != null)
                handler(this, new PropertyChangedEventArgs(propertyName));
        }
        public MainWindow()
        {
            InitializeComponent();
            this.DataContext = this;
            firstLoad();
        }
        #region properties
        private bool _IsConnected;
        public bool IsConnected
        {
            get => _IsConnected;
            set 
            {   _IsConnected = value;
                OnPropertyChanged();
            }
        }

        private bool _IsStarted;
        public bool IsStarted
        {
            get => _IsStarted;
            set 
            {
                _IsStarted = value;
                OnPropertyChanged();
            }
        }

        private bool _IsPause;
        public bool IsPause
        {
            get => _IsPause;
            set
            {
                _IsPause = value;
                OnPropertyChanged();
            }
        }

        private bool _IsGcodeSelected;
        public bool IsGcodeSelected
        {
            get => _IsGcodeSelected;
            set
            {
                _IsGcodeSelected = value;
                OnPropertyChanged();
            }
        }
        #endregion

        #region fields
        public static UsbDevice myUsbDevice;
        public static UsbDeviceFinder myUsbFinder = new UsbDeviceFinder(1115, 22370);
        UsbEndpointReader reader;
        UsbEndpointWriter writer;

        #endregion

        #region methods
        void firstLoad()
        {
            IsConnected = false;
            IsStarted = false;
            IsGcodeSelected =false;
        }

        private void addToTextBox(string input)
        {            
        }
        #endregion

        #region events
        private void CloseApp(object sender, MouseButtonEventArgs e)
        {
            this.Close();
        }
        private void MinimizeApp(object sender, MouseButtonEventArgs e)
        {
            WindowState = WindowState.Minimized;
        }

        private void DragWindow(object sender, MouseButtonEventArgs e)
        {
            base.OnMouseLeftButtonDown(e);
            this.DragMove();
        }
        private void Connect(object sender, MouseButtonEventArgs e)
        {
            // check status start
            if (IsStarted)
                return;
            if (!IsConnected) //kết nối
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
                    IsConnected = true;
                }
                catch
                {
                    System.Windows.MessageBox.Show("error");
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
                IsConnected = false;
            }
        }

        private void OnRxEndPointData(object sender, EndpointDataEventArgs e)
        {
            //Action<string> Action = addToTextBox;
            //this.BeginInvoke(Action, (Encoding.Default.GetString(e.Buffer, 0, e.Count)));
            addToTextBox(Encoding.Default.GetString(e.Buffer, 0, e.Count));
        }

        private void Start(object sender, MouseButtonEventArgs e)
        {
            // check connect
            if (!IsConnected)
                return;
            if (!IsGcodeSelected)
                return;
            // process 
            if (!IsStarted)
            {
                IsStarted = true;
            }
            else
            {
                IsStarted = false;
                IsPause = false;
            }
        }

        private void Pause(object sender, MouseButtonEventArgs e)
        {
            // check status start
            if (!IsStarted)
                return;
            if (!IsPause)
            {
                IsPause = true;
            }
            else 
            {
                IsPause= false;
            }
        }

        private void SelectGcode(object sender, MouseButtonEventArgs e)
        {
            if (!IsConnected)
                return;
            if (IsStarted)
                return;
            // Create OpenFileDialog 
            Microsoft.Win32.OpenFileDialog dlg = new Microsoft.Win32.OpenFileDialog();

            // Set filter for file extension and default file extension 
            dlg.DefaultExt = ".txt";
            dlg.Filter = "txt Files (*.txt)|*.txt|JPEG Files (*.jpeg)|*.jpeg|PNG Files (*.png)|*.png|JPG Files (*.jpg)|*.jpg|GIF Files (*.gif)|*.gif";

            // Display OpenFileDialog by calling ShowDialog method 
            Nullable<bool> result = dlg.ShowDialog();
            if (result == true)
            {
                // Open document 
                string filename = dlg.FileName;
                Gcode.Text = File.ReadAllText(filename);
                IsGcodeSelected = true;
            }
        }
        #endregion
    }
}
