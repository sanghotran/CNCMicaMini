﻿#pragma checksum "..\..\MainWindow.xaml" "{8829d00f-11b8-4213-878b-770e8597ac16}" "25247502B8C7F7E1D9B82912D9F634E0760F6352F2FF98E172ECD9D8BABED238"
//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.42000
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

using CNCMicaMiniWPF;
using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Automation;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Markup;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Media.TextFormatting;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Shell;


namespace CNCMicaMiniWPF {
    
    
    /// <summary>
    /// MainWindow
    /// </summary>
    public partial class MainWindow : System.Windows.Window, System.Windows.Markup.IComponentConnector {
        
        
        #line 221 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock Gcode;
        
        #line default
        #line hidden
        
        
        #line 419 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.ScrollViewer DebugScroll;
        
        #line default
        #line hidden
        
        
        #line 420 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock debug;
        
        #line default
        #line hidden
        
        
        #line 426 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.ScrollViewer LogScroll;
        
        #line default
        #line hidden
        
        
        #line 427 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock log;
        
        #line default
        #line hidden
        
        
        #line 584 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox Kp;
        
        #line default
        #line hidden
        
        
        #line 595 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox Ki;
        
        #line default
        #line hidden
        
        
        #line 606 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox Kd;
        
        #line default
        #line hidden
        
        
        #line 665 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox setpoint;
        
        #line default
        #line hidden
        
        private bool _contentLoaded;
        
        /// <summary>
        /// InitializeComponent
        /// </summary>
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        public void InitializeComponent() {
            if (_contentLoaded) {
                return;
            }
            _contentLoaded = true;
            System.Uri resourceLocater = new System.Uri("/CNCMicaMini;component/mainwindow.xaml", System.UriKind.Relative);
            
            #line 1 "..\..\MainWindow.xaml"
            System.Windows.Application.LoadComponent(this, resourceLocater);
            
            #line default
            #line hidden
        }
        
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        [System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Design", "CA1033:InterfaceMethodsShouldBeCallableByChildTypes")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1800:DoNotCastUnnecessarily")]
        void System.Windows.Markup.IComponentConnector.Connect(int connectionId, object target) {
            switch (connectionId)
            {
            case 1:
            
            #line 79 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.DockPanel)(target)).PreviewMouseLeftButtonDown += new System.Windows.Input.MouseButtonEventHandler(this.DragWindow);
            
            #line default
            #line hidden
            return;
            case 2:
            
            #line 85 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Image)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.CloseApp);
            
            #line default
            #line hidden
            return;
            case 3:
            
            #line 102 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Image)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.MinimizeApp);
            
            #line default
            #line hidden
            return;
            case 4:
            
            #line 124 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Image)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.ControlPage);
            
            #line default
            #line hidden
            return;
            case 5:
            
            #line 141 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Image)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.DebugPage);
            
            #line default
            #line hidden
            return;
            case 6:
            
            #line 168 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.SelectImage);
            
            #line default
            #line hidden
            return;
            case 7:
            this.Gcode = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 8:
            
            #line 228 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.SelectGcode);
            
            #line default
            #line hidden
            return;
            case 9:
            
            #line 271 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.Connect);
            
            #line default
            #line hidden
            return;
            case 10:
            
            #line 311 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.Start);
            
            #line default
            #line hidden
            return;
            case 11:
            
            #line 368 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.Pause);
            
            #line default
            #line hidden
            return;
            case 12:
            this.DebugScroll = ((System.Windows.Controls.ScrollViewer)(target));
            return;
            case 13:
            this.debug = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 14:
            this.LogScroll = ((System.Windows.Controls.ScrollViewer)(target));
            return;
            case 15:
            this.log = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 16:
            
            #line 437 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.Home);
            
            #line default
            #line hidden
            return;
            case 17:
            
            #line 484 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Image)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.Setting);
            
            #line default
            #line hidden
            return;
            case 18:
            
            #line 497 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.X);
            
            #line default
            #line hidden
            return;
            case 19:
            
            #line 517 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.Y);
            
            #line default
            #line hidden
            return;
            case 20:
            
            #line 537 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.Z);
            
            #line default
            #line hidden
            return;
            case 21:
            
            #line 557 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.Calib);
            
            #line default
            #line hidden
            return;
            case 22:
            this.Kp = ((System.Windows.Controls.TextBox)(target));
            return;
            case 23:
            this.Ki = ((System.Windows.Controls.TextBox)(target));
            return;
            case 24:
            this.Kd = ((System.Windows.Controls.TextBox)(target));
            return;
            case 25:
            
            #line 614 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.SendPID);
            
            #line default
            #line hidden
            return;
            case 26:
            this.setpoint = ((System.Windows.Controls.TextBox)(target));
            return;
            case 27:
            
            #line 673 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Grid)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.SendCalib);
            
            #line default
            #line hidden
            return;
            case 28:
            
            #line 737 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.TextBlock)(target)).PreviewMouseDown += new System.Windows.Input.MouseButtonEventHandler(this.Ok_Warning);
            
            #line default
            #line hidden
            return;
            }
            this._contentLoaded = true;
        }
    }
}

