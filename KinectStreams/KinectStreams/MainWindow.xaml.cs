using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Drawing;

namespace KinectStreams
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Members
        
        Mode _mode = Mode.Color;

        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;
       
        private WriteableBitmap BodyBitmap = null;

        String linesBody = "";
        String linesHand = " ";
        String savepathRightHand = "C:\\Users\\x1c\\Desktop\\1.txt";


        private const int BytesPerPixel = 4;
        private static readonly uint[] BodyColor =
        {
            0x0000FF00,
            0x00FF0000,
            0xFFFF4000,
            0x40FFFF00,
            0xFF40FF00,
            0xFF808000,
        };
        bool _displayBody = false;
        bool _SaveJointData = false;
        bool _SaveJointHandData = false;
        
        private UInt16 DataNum = 0;
        #endregion

        #region Constructor

        public MainWindow()
        {
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(savepathRightHand, true))// MAKE SURE YOU MODIFY THE savepathRightHand ABOVE!!!
            {
                file.WriteLine("Working...");
            }
            InitializeComponent();
        }

        #endregion

        #region Event handlers

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();


            //this.BodyBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);


            // create the bitmap to display
            this.BodyBitmap = new WriteableBitmap(_sensor.ColorFrameSource.FrameDescription.Width,
                _sensor.ColorFrameSource.FrameDescription.Height, 
                96.0, 96.0, PixelFormats.Bgr32, null);

            if (_sensor != null)
            {
                _sensor.Open();
                // create the bitmap to display
                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
            }
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
        }

        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();
            Joint joint_hand;
            // Color
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == Mode.Color)
                    {
                        FrameDescription colorFrameDescription = frame.FrameDescription;           
                         camera.Source = frame.ToBitmap();
                         using (KinectBuffer colorBuffer = frame.LockRawImageBuffer())
                         {
                             this.BodyBitmap.Lock();
                             frame.CopyConvertedFrameDataToIntPtr(
                                  this.BodyBitmap.BackBuffer,
                                  (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                  ColorImageFormat.Bgra);

                             this.BodyBitmap.AddDirtyRect(new Int32Rect(0, 0, this.BodyBitmap.PixelWidth, this.BodyBitmap.PixelHeight));
                             this.BodyBitmap.Unlock();
                         }
                    }     
                }
            }

            // Depth
            using (var frame = reference.DepthFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == Mode.Depth)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }

            // Infrared
            using (var frame = reference.InfraredFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == Mode.Infrared)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }

            // Body
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    canvas.Children.Clear();

                    _bodies = new Body[frame.BodyFrameSource.BodyCount];

                    frame.GetAndRefreshBodyData(_bodies);

                    foreach (var body in _bodies)
                    {                     
                        if (body != null)
                        {
                            if (body.IsTracked)
                            {
                                // Draw skeleton.
                                if (_displayBody)
                                {
                                    canvas.DrawSkeleton(body);
                                }

                                if (_SaveJointData)
                                {
                                    linesBody += DataNum++;
                                    foreach (Joint joint in body.Joints.Values)
                                    {
                                           String linesBody_temp = String.Format(",{0},{1},{2}",
                                           joint.Position.X.ToString(),joint.Position.Y.ToString(), joint.Position.Z.ToString());
                                           linesBody = linesBody + linesBody_temp;

                                           if (joint.JointType == JointType.HandRight) {
                                                //save one frame hand data 
                                               joint_hand = joint.ScaleTo(canvas.ActualWidth, canvas.ActualHeight);
                                               String linesHand_temp = String.Format("{0},{1},{2}",
                                               joint_hand.Position.X.ToString(), joint_hand.Position.Y.ToString(),joint_hand.Position.Z.ToString());
                                               linesHand = linesHand_temp; 
                                           }                                    
                                    }
                                 
                                    linesBody += Environment.NewLine;

                                    #region Singlepoint
                                    /*   
                                    switch (joint.JointType) {
                                        case JointType.HandRight: 
                                            String linesBody_temp = String.Format("{0}, {1}, {2}" + Environment.NewLine,
                                             joint.Position.X.ToString(), joint.Position.Y.ToString(), joint.Position.Z.ToString());
                                            linesBody = linesBody + linesBody_temp;

                                            break;
                                        case JointType.WristRight:
                                            String linesLeftHand_temp = String.Format("{0}, {1}, {2}" +  Environment.NewLine,                    
                                            joint.Position.X.ToString(), joint.Position.Y.ToString(),joint.Position.Z.ToString());
                                            linesWristRight = linesWristRight + linesLeftHand_temp;
                                            break;
                                    }
                                     */
                                    #endregion

                                }
                            }
                        }
                    }
                }
            }
        }

        private void Color_Click(object sender, RoutedEventArgs e)
        {
            _mode = Mode.Color;
        }

        private void Depth_Click(object sender, RoutedEventArgs e)
        {
            _mode = Mode.Depth;
        }

        private void Infrared_Click(object sender, RoutedEventArgs e)
        {
            _mode = Mode.Infrared;
        }

        private void Body_Click(object sender, RoutedEventArgs e)
        {
            _displayBody = !_displayBody;
        }      

        private void ScreenShot_Click(object sender, RoutedEventArgs e)
        {
            GetScreenSkeleton();

            if (linesHand != " ") {
                txtCreator(linesHand, JointType.HandRight);
                linesHand = "";
            }
          
           // GetScreen();
        }

        void txtCreator(String line, JointType jonttype)
        {

            string txtTime = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);
            string myData = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
            string myDtapath = System.IO.Path.Combine(myData, "KinectScreenshot-Gesture-" + txtTime + ".txt");

            string handData = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
            string handDatapath = System.IO.Path.Combine(handData, "KinectScreenshot-Hand-" + txtTime + ".txt");

            //保存全部的数据
            if (jonttype == JointType.Head)
            {
                if (line == " ") return;
                System.IO.File.WriteAllText(myDtapath, String.Empty);
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(myDtapath, true))
                {
                    file.WriteLine(line);
                }         
            }
            //保存缩放后手部的坐标数据
            if (jonttype == JointType.HandRight)
            {
                System.IO.File.WriteAllText(handDatapath, String.Empty);
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(handDatapath, true))
                {
                    file.WriteLine(line);
                }
            }
        }



        public void GetScreenSkeleton() {
            // create a png bitmap encoder which knows how to save a .png file
            BitmapEncoder encoder = new PngBitmapEncoder();
            // create frame from the writable bitmap and add to encoder
            encoder.Frames.Add(BitmapFrame.Create(this.BodyBitmap));
            string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);
            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
            // write the new file to disk
            string path = System.IO.Path.Combine(myPhotos, "KinectScreenshot-Gesture-" + time + ".png");
            try
            {
                // FileStream is IDisposable
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encoder.Save(fs);
                    fs.Close();
                }
            }
            catch (IOException)
            {
            }
        }
        public void GetScreen()
        {
            //获取整个屏幕图像,不包括任务栏

            RenderTargetBitmap targetBitmap = new RenderTargetBitmap((int)Width, (int)Height, 96d, 96d, PixelFormats.Default);
            targetBitmap.Render(this);
            PngBitmapEncoder saveEncoder = new PngBitmapEncoder();
            saveEncoder.Frames.Add(BitmapFrame.Create(targetBitmap));
            string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);
            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
            // write the new file to disk
            string path = System.IO.Path.Combine(myPhotos, "KinectScreenshot-Gesture-" + time + ".png");
            //string tempFile = System.IO.Path.GetTempFileName() + ".png";
            System.IO.FileStream fs = System.IO.File.Open(path, System.IO.FileMode.OpenOrCreate);
            saveEncoder.Save(fs);
            fs.Close();
    
        }

        private void StartSave_Click(object sender, RoutedEventArgs e)
        {
            _SaveJointData = true;
        }
        private void StopSave_Click(object sender, RoutedEventArgs e)
        {
            _SaveJointData = false;
            if (linesBody != " ")
            {
                txtCreator(linesBody, JointType.Head);
                linesBody = "";
                DataNum = 0;
            }
        }
    }


      #endregion
       
    public enum Mode
    {
        Color,
        Depth,
        Infrared
    }
}
