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
            { _IsConnected = value;
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

        private Visibility _IsSettingPageShow;
        public Visibility IsSettingPageShow
        {
            get => _IsSettingPageShow;
            set
            {
                _IsSettingPageShow = value;
                OnPropertyChanged();
            }
        }

        private bool _IsX;
        public bool IsX
        {
            get => _IsX;
            set
            {
                _IsX = value;
                OnPropertyChanged();
            }
        }

        private bool _IsY;
        public bool IsY
        {
            get => _IsY;
            set
            {
                _IsY = value;
                OnPropertyChanged();
            }
        }

        private bool _IsZ;
        public bool IsZ
        {
            get => _IsZ;
            set
            {
                _IsZ = value;
                OnPropertyChanged();
            }
        }

        private bool _IsCalib;
        public bool IsCalib
        {
            get => _IsCalib;
            set
            {
                _IsCalib = value;
                OnPropertyChanged();
            }
        }

        private Visibility _IsInputPIDPageShow;
        public Visibility IsInputPIDPageShow
        {
            get => _IsInputPIDPageShow;
            set
            {
                _IsInputPIDPageShow = value;
                OnPropertyChanged();
            }
        }

        private Visibility _IsCalibPageShow;
        public Visibility IsCalibPageShow
        {
            get => _IsCalibPageShow;
            set
            {
                _IsCalibPageShow = value;
                OnPropertyChanged();
            }
        }

        private Visibility _IsSettingCNCPageShow;
        public Visibility IsSettingCNCPageShow
        {
            get => _IsSettingCNCPageShow;
            set
            {
                _IsSettingCNCPageShow = value;
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
        string ResendBuff;
        string[] ReceiveData;
        string[] ShowData;
        string Calib_data;
        int point;
        int send_count;

        string debug_data;
        bool debug_flag = false;

        string log_data;
        bool log_flag = false;

        public struct PID
        {
            public string Kp;
            public string Ki;
            public string Kd;
            public string send;
        }
        public PID x_PID;
        public PID y_PID;
        public PID z_PID;

        #endregion

        #region methods
        void firstLoad()
        {
            IsConnected = false;
            IsStarted = false;
            IsGcodeSelected =false;
            IsDebug = false;
            IsWarningBoxShow = Visibility.Hidden;
            IsDebugPageShow = Visibility.Hidden;
            IsSettingPageShow = Visibility.Hidden;
            IsInputPIDPageShow = Visibility.Hidden;
            IsCalibPageShow = Visibility.Hidden;

            // start new thread for print debug
            DebugThread = new Thread(new ThreadStart(Debug));
            DebugThread.IsBackground = true;
            DebugThread.Start();
        }
        private void SendData(string input, int num)
        {
            try
            {
                int bytesWritten;
                ResendBuff = input;
                input = num.ToString() + ' ' + input;
                if ((input.Length % 2) == 0)
                {
                    input += ' ';
                }
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
            // auto scroll to end of debug
            if (DebugScroll.VerticalOffset == DebugScroll.ScrollableHeight)
            {
                DebugScroll.ScrollToEnd();
            }
            // auto scroll to end of log
            if (LogScroll.VerticalOffset == LogScroll.ScrollableHeight)
            {
                LogScroll.ScrollToEnd();
            }

            //split data
            ReceiveData = input.Split(' ');

            // show data
            try
            {
                ShowData = input.Split('_');

                // print to log
                log_data = ShowData[0] + '\n';
                log_flag = true;

                // print to debug
                debug_data = ShowData[1] + '\n';
                debug_flag = true;
            }
            catch
            {

            }

            // check ACK from CNC
            if ( ReceiveData[0] == "ACK")
            {
                send_count++;
                // Check Command form CNC
                if (ReceiveData[1] == "R")
                {
                    if (IsStarted)
                    {
                        if (IsPause)
                        {
                            SendData("P", send_count);
                            return;
                        }
                        while (!GcodeBuff[point].Contains("G0"))
                        {
                            point++;
                            if (point == GcodeBuff.Length)
                            {
                                IsStarted = false;
                                SendData("STP", send_count);
                                return;
                            }
                        }
                        SendData(GcodeBuff[point], send_count);
                        point++;                        
                    }
                    else
                    {
                        SendData("STP", send_count);
                    }
                }

                if (ReceiveData[1] == "STP")
                    send_count = 0;
                return;
            }

            // check NAK from CNC
            if (ReceiveData[0] == "NAK")
            {
                SendData(ResendBuff, send_count);
                return;
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

                if (log_flag)
                {
                    log_flag = false;
                    log.Dispatcher.Invoke(() => log.Text += log_data); ;
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
                point = 0;
                SendData("STR", send_count);
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
                SendData("R", send_count);
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
        private void Setting(object sender, MouseButtonEventArgs e)
        {
            if (IsSettingPageShow == Visibility.Visible)
            {
                IsSettingPageShow = Visibility.Hidden;
                IsSettingCNCPageShow = Visibility.Visible;
            }
            else
            {
                IsSettingPageShow = Visibility.Visible;
                IsSettingCNCPageShow = Visibility.Hidden;
            }
        }
        private void Calib(object sender, MouseButtonEventArgs e)
        {
            IsCalib = true;
            IsX = false;
            IsY = false;
            IsZ = false;
            IsInputPIDPageShow = Visibility.Hidden;
            IsCalibPageShow = Visibility.Visible;
        }
        private void X(object sender, MouseButtonEventArgs e)
        {
            IsX = true;
            IsY = false;
            IsZ = false;
            IsCalib = false;
            IsInputPIDPageShow = Visibility.Visible;
            IsCalibPageShow = Visibility.Hidden;
            Kp.Text = x_PID.Kp;
            Ki.Text = x_PID.Ki;
            Kd.Text = x_PID.Kd;
        }
        private void Y(object sender, MouseButtonEventArgs e)
        {
            IsY = true;
            IsX = false;
            IsZ = false;
            IsCalib = false;
            IsInputPIDPageShow = Visibility.Visible;
            IsCalibPageShow = Visibility.Hidden;
            Kp.Text = y_PID.Kp;
            Ki.Text = y_PID.Ki;
            Kd.Text = y_PID.Kd;
        }
        private void Z(object sender, MouseButtonEventArgs e)
        {
            IsZ = true;
            IsX = false;
            IsY = false;
            IsCalib = false;
            IsInputPIDPageShow = Visibility.Visible;
            IsCalibPageShow = Visibility.Hidden;
            Kp.Text = z_PID.Kp;
            Ki.Text = z_PID.Ki;
            Kd.Text = z_PID.Kd;
        }
        private void SendPID(object sender, MouseButtonEventArgs e)
        {
            if (!IsConnected)
                return;
            if (IsStarted)
                return;
            if(IsX)
            {
                x_PID.Kp = Kp.Text;
                x_PID.Ki = Ki.Text;
                x_PID.Kd = Kd.Text;
                x_PID.send = "IX " + x_PID.Kp + " " + x_PID.Ki + " " + x_PID.Kd;
                SendData(x_PID.send, send_count);
            }
            if (IsY)
            {
                y_PID.Kp = Kp.Text;
                y_PID.Ki = Ki.Text;
                y_PID.Kd = Kd.Text;
                y_PID.send = "IY " + y_PID.Kp + " " + y_PID.Ki + " " + y_PID.Kd;
                SendData(y_PID.send, send_count);
            }
            if (IsZ)
            {
                z_PID.Kp = Kp.Text;
                z_PID.Ki = Ki.Text;
                z_PID.Kd = Kd.Text;
                z_PID.send = "IZ " + z_PID.Kp + " " + z_PID.Ki + " " + z_PID.Kd;
                SendData(z_PID.send, send_count);
            }
        }
        private void SendCalib(object sender, MouseButtonEventArgs e)
        {
            if (!IsConnected)
                return;
            if (IsStarted)
                return;
            Calib_data = "C " + setpoint.Text;
            SendData(Calib_data, send_count);
        }
        private void Home(object sender, MouseButtonEventArgs e)
        {
            if (!IsConnected)
                return;
            if (IsStarted)
                return;
            SendData("H " + thickness.Text , send_count);
        }
        #endregion


    }
}
