using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
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

        private bool _IsDebug;
        public bool IsDebug
        {
            get => _IsDebug;
            set
            {
                _IsDebug = value;
                OnPropertyChanged();
            }
        }

        private Visibility _IsWarningBoxShow;
        public Visibility IsWarningBoxShow
        {
            get => _IsWarningBoxShow;
            set
            {
                _IsWarningBoxShow = value;
                OnPropertyChanged();
            }
        }

        private Visibility _IsControlPageShow;
        public Visibility IsControlPageShow
        {
            get => _IsControlPageShow;
            set
            {
                _IsControlPageShow = value;
                OnPropertyChanged();
            }
        }

        private Visibility _IsDebugPageShow;
        public Visibility IsDebugPageShow
        {
            get => _IsDebugPageShow;
            set
            {
                _IsDebugPageShow = value;
                OnPropertyChanged();
            }
        }

        #endregion

        #region fields
        public static UsbDevice myUsbDevice;
        public static UsbDeviceFinder myUsbFinder = new UsbDeviceFinder(1115, 22370);
        UsbEndpointReader reader;
        UsbEndpointWriter writer;

        private Thread DebugThread; 

        string[] GcodeBuff;
        string debug_data;
        int point;
        bool debug_flag = false;

        #endregion

        #region methods
        void firstLoad()
        {
            IsConnected = false;
            IsStarted = false;
            IsGcodeSelected =false;
            IsDebug = false;
            IsWarningBoxShow = Visibility.Hidden;
            IsControlPageShow = Visibility.Visible;
            IsDebugPageShow = Visibility.Hidden;

            DebugThread = new Thread(new ThreadStart(Debug));
            DebugThread.IsBackground = true;
            DebugThread.Start();
        }
        private void SendData(string input)
        {
            try
            {
                int bytesWritten;
                writer.Write(Encoding.Default.GetBytes(input), 1000, out bytesWritten);
            }
            catch (Exception)
            {
                IsWarningBoxShow = Visibility.Visible;
                return;
            }
        }
        private void ProcessReceiveData(string input)
        {
            debug_data = input + "\n";
            debug_flag = true;            
            //debug.Text += input + "\n";
            if (input == "R")
            {
                if (IsStarted)
                {
                    if (IsPause)
                    {
                        SendData("P");
                        return;
                    }
                    if((GcodeBuff[point].Length % 2) == 0)
                    {
                        GcodeBuff[point] += '0';
                    }
                    SendData(GcodeBuff[point]);
                    point++;
                    if ( point == GcodeBuff.Length)
                        IsStarted = false;
                }
                else
                {
                    SendData("STP");
                }
            }
        }
        private void Debug()
        {
            while( true)
            {
                if (debug_flag)
                {
                    debug_flag = false;
                    debug.Dispatcher.Invoke( ()=>  debug.Text += debug_data);
                }
            }
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
        private void ControlPage(object sender, MouseButtonEventArgs e)
        {
            IsControlPageShow = Visibility.Visible;
            IsDebugPageShow = Visibility.Hidden;
        }
        private void DebugPage(object sender, MouseButtonEventArgs e)
        {
            IsDebugPageShow = Visibility.Visible;
            IsControlPageShow = Visibility.Hidden;
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
                    IsWarningBoxShow = Visibility.Visible;
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
            Action<string> Action = ProcessReceiveData;
            this.Dispatcher.Invoke(Action, (Encoding.Default.GetString(e.Buffer, 0, e.Count)));
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
                SendData("STR");
                point = 0;
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
                SendData("R");
                IsPause= false;
            }
        }
        private void SelectImage(object sender, MouseButtonEventArgs e)
        {
            if (!IsConnected)
                return;
            if (IsStarted)
                return;
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
                string data;
                data = File.ReadAllText(filename);
                Gcode.Text = data;
                GcodeBuff = Gcode.Text.Split('\n');                
                IsGcodeSelected = true;
            }
        }
        private void Ok_Warning(object sender, MouseButtonEventArgs e)
        {
            IsWarningBoxShow = Visibility.Hidden;
        }

        #endregion
    }
}
