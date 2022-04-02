using System;
using System.IO.Ports;

namespace CNCMicaMini
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.cbo_Select_Com = new System.Windows.Forms.ComboBox();
            this.COM = new System.IO.Ports.SerialPort(this.components);
            this.txt_Test = new System.Windows.Forms.TextBox();
            this.btn_Send = new System.Windows.Forms.Button();
            this.btn_Connect = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            // 
            // cbo_Select_Com
            // 
            this.cbo_Select_Com.FormattingEnabled = true;
            this.cbo_Select_Com.Location = new System.Drawing.Point(12, 12);
            this.cbo_Select_Com.Name = "cbo_Select_Com";
            this.cbo_Select_Com.Size = new System.Drawing.Size(121, 21);
            this.cbo_Select_Com.TabIndex = 0;
            // 
            // COM
            // 
            this.COM.PortName = "SerialPort";
            // 
            // txt_Test
            // 
            this.txt_Test.Location = new System.Drawing.Point(12, 67);
            this.txt_Test.Multiline = true;
            this.txt_Test.Name = "txt_Test";
            this.txt_Test.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.txt_Test.Size = new System.Drawing.Size(294, 136);
            this.txt_Test.TabIndex = 1;
            // 
            // btn_Send
            // 
            this.btn_Send.Location = new System.Drawing.Point(150, 12);
            this.btn_Send.Name = "btn_Send";
            this.btn_Send.Size = new System.Drawing.Size(75, 23);
            this.btn_Send.TabIndex = 2;
            this.btn_Send.Text = "Send";
            this.btn_Send.UseVisualStyleBackColor = true;
            this.btn_Send.Click += new System.EventHandler(this.btn_Send_Click);
            // 
            // btn_Connect
            // 
            this.btn_Connect.Location = new System.Drawing.Point(12, 38);
            this.btn_Connect.Name = "btn_Connect";
            this.btn_Connect.Size = new System.Drawing.Size(75, 23);
            this.btn_Connect.TabIndex = 3;
            this.btn_Connect.Text = "Connect";
            this.btn_Connect.UseVisualStyleBackColor = true;
            this.btn_Connect.Click += new System.EventHandler(this.btn_Connect_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(318, 215);
            this.Controls.Add(this.btn_Connect);
            this.Controls.Add(this.btn_Send);
            this.Controls.Add(this.txt_Test);
            this.Controls.Add(this.cbo_Select_Com);
            this.Name = "Form1";
            this.Text = "Form1";
            this.ResumeLayout(false);
            this.PerformLayout();

        }


        #endregion

        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.ComboBox cbo_Select_Com;
        private System.IO.Ports.SerialPort COM;
        private System.Windows.Forms.TextBox txt_Test;
        private System.Windows.Forms.Button btn_Send;
        private System.Windows.Forms.Button btn_Connect;
    }
}

