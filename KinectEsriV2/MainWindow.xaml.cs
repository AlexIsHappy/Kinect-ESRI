using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using Microsoft.Kinect.Input;
using Microsoft.Kinect.Toolkit;
using Microsoft.Kinect.Wpf.Controls;
using ESRI.ArcGIS.Client;
using ESRI.ArcGIS.Client.Tasks;

namespace KinectEsriV2
{
    public struct Vector2
    {
        public float X;
        public float Y;

        public static Vector2 Zero
        {
            get
            {
                return new Vector2(0, 0);
            }
        }

        public Vector2(float x, float y)
        {
            this.X = x;
            this.Y = y;
        }

        public float Length
        {
            get
            {
                return (float)Math.Sqrt(X * X + Y * Y);
            }
        }

        public static double Distance(Vector2 left, Vector2 right)
        {
            return Math.Sqrt(Math.Pow((left.X - right.X), 2) + Math.Pow((left.Y - right.Y), 2));
        }

        public static Vector2 operator -(Vector2 left, Vector2 right)
        {
            return new Vector2(left.X - right.X, left.Y - right.Y);
        }
        public static Vector2 operator +(Vector2 left, Vector2 right)
        {
            return new Vector2(left.X + right.X, left.Y + right.Y);
        }
        public static Vector2 operator *(Vector2 left, float value)
        {
            return new Vector2(left.X * value, left.Y * value);
        }
        public static Vector2 operator *(float value, Vector2 left)
        {
            return left * value;
        }
        public static Vector2 operator /(Vector2 left, float value)
        {
            return new Vector2(left.X / value, left.Y / value);
        }

    }

    public struct Vector3
    {
        public float X;
        public float Y;
        public float Z;

        public static Vector3 Zero
        {
            get
            {
                return new Vector3(0, 0, 0);
            }
        }

        public Vector3(float x, float y, float z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        public float Length
        {
            get
            {
                return (float)Math.Sqrt(X * X + Y * Y + Z * Z);
            }
        }
        public static double Distance(Vector3 left, Vector3 right)
        {
            return Math.Sqrt(Math.Pow((left.X - right.X), 2) + Math.Pow((left.Y - right.Y), 2) + Math.Pow((left.Z - right.Z), 2));
        }

        public static Vector3 operator -(Vector3 left, Vector3 right)
        {
            return new Vector3(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
        }
        public static Vector3 operator +(Vector3 left, Vector3 right)
        {
            return new Vector3(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
        }
        public static Vector3 operator *(Vector3 left, float value)
        {
            return new Vector3(left.X * value, left.Y * value, left.Z * value);
        }
        public static Vector3 operator *(float value, Vector3 left)
        {
            return left * value;
        }
        public static Vector3 operator /(Vector3 left, float value)
        {
            return new Vector3(left.X / value, left.Y / value, left.Z / value);
        }
    }



    public partial class MainWindow : Window
    {
        KinectSensor sensor = null;
        MultiSourceFrameReader reader = null;
        Body[] bodies= new Body[0];
        List<Body> currentBodies = new List<Body>();
        bool ableToControl = false;

        //Last hand pointer
        KinectPointerPoint handPointer = null;

        //Hands data
        List<ContextPoint> Positions = new List<ContextPoint>();

        // Map staff 
        ESRI.ArcGIS.Client.ClassBreaksRenderer MyRenderer = new ESRI.ArcGIS.Client.ClassBreaksRenderer();
        ESRI.ArcGIS.Client.Geometry.Envelope currentExtent = new ESRI.ArcGIS.Client.Geometry.Envelope();
        ESRI.ArcGIS.Client.Geometry.Envelope initialExtent = new ESRI.ArcGIS.Client.Geometry.Envelope();
        ESRI.ArcGIS.Client.Geometry.Envelope startExtent = new ESRI.ArcGIS.Client.Geometry.Envelope();
        ESRI.ArcGIS.Client.Geometry.MapPoint mapCenter = new ESRI.ArcGIS.Client.Geometry.MapPoint();
        double initialResolution;


        public MainWindow()
        {
            InitializeComponent();
            
           
            sensor = KinectSensor.GetDefault();

            reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color |
                                             FrameSourceTypes.Depth |
                                             FrameSourceTypes.Body);
        }


        private void ControlsBasicsWindow_Loaded(object sender, RoutedEventArgs e)
        {
            initialResolution = MyMap.Resolution;
            initialExtent = MyMap.Extent;
            currentExtent = MyMap.Extent;
        }

        private void Grid_Loaded(object sender, RoutedEventArgs e)
        {
            if (sensor != null)
            {
                sensor.Open();
            }

            reader.MultiSourceFrameArrived += reader_MultiSourceFrameArrived;

            var kinectCoreWindow = KinectCoreWindow.GetForCurrentThread();
            kinectCoreWindow.PointerMoved += this.kinectCoreWindow_PointerMoved;

           
        }

        private void kinectCoreWindow_PointerMoved(object sender, KinectPointerEventArgs e)
        {
            if (bodies.Length != null && bodies.Length != 0)
            {
                handPointer = e.CurrentPoint;
            }
        }

        void reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // Get a reference to the multi-frame
            var reference = e.FrameReference.AcquireFrame();

            // Open color frame
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    ColorImage.Source = ToBitmap(frame);
                }
            }

            // Open depth frame
            using (var frame = reference.DepthFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    // Do something with the frame...
                }
            }

            // Open body frame
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    bodies = new Body[frame.BodyCount];
                    frame.GetAndRefreshBodyData(bodies);
                }

                Body trackedBody = FirstTrackedBody(bodies);
                currentBodies.Add(trackedBody);

                // Animation stuff - Implement this later
                if (trackedBody== null)
                {
                    // Implement this later
                }

                else
                {
                   // Implement this later
                    ableToControl = true;
                }


                // COntrol of map Implementation

                if (ableToControl == true)
                {
                  if (handPointer == null || handPointer.Properties.BodyTrackingId != trackedBody.TrackingId)
                     {
                         return;
                     }

                    JointType currentHand = 0;

                    try
                    {
                        if (handPointer.Properties.HandType == HandType.LEFT)
                            currentHand = JointType.HandLeft;
                        else if (handPointer.Properties.HandType == HandType.RIGHT)
                            currentHand = JointType.HandRight;
                    }
                    catch (Exception ex1)
                    {

                    }

                    // And FINALLY perform the action on the map
                    if (handPointer != null && trackedBody.Joints.Where(j => j.Key == currentHand).FirstOrDefault().Value.TrackingState == TrackingState.Tracked)
                    {
                        ESRI.ArcGIS.Client.Geometry.Envelope newEx = new ESRI.ArcGIS.Client.Geometry.Envelope();
                        /*newEx = MyMap.Extent;
                        if (MyMap.Extent.YMax >= 88)
                        {
                            newEx.YMax = 85;
                            newEx.YMin = MyMap.Extent.YMin + 1;
                            newEx.XMax = MyMap.Extent.XMax;
                            newEx.XMin = MyMap.Extent.XMin;
                        }
                        else if (MyMap.Extent.YMin <= -88)
                        {
                            newEx.YMin = -85;
                            newEx.YMax = MyMap.Extent.YMax - 1;
                            newEx.XMin = MyMap.Extent.XMin;
                            newEx.XMax = MyMap.Extent.XMax;
                        }
                        MyMap.Extent = newEx;*/

                        // Track the right amount of tracked data
                       /* if (actions.Count > 11)
                            actions.RemoveAt(0);

                        if (zooms.Count > 10)
                            zooms.RemoveAt(0);

                        if (handPointerX.Count > 3 && handPointerY.Count > 3)
                        {
                            handPointerX.RemoveAt(0);
                            handPointerY.RemoveAt(0);
                        }
                        */
                        MoveTheMapPointer(handPointer, 5, trackedBody);
                    }
                }

            }
        }

        //Control the Map
        public void MoveTheMapPointer(KinectPointerPoint handPoiter, int numbOfPosit, Body skeleton)
        {
            ESRI.ArcGIS.Client.Geometry.MapPoint currentPoint = new ESRI.ArcGIS.Client.Geometry.MapPoint();
            ESRI.ArcGIS.Client.Geometry.MapPoint lastPoint = new ESRI.ArcGIS.Client.Geometry.MapPoint();
            double distanceX = 0;
            double distanceY = 0;
            double distanceZ = 0;
            // real positions
            double realDistX = 0;
            double realDistY = 0;
            double realDistZ = 0;
            double distanceZoom = 0;

            Positions.Add(new ContextPoint()
            {
                X = handPoiter.Position.X,
                Y = handPoiter.Position.Y,
                Time = DateTime.Now
            });

            if (Positions.Count > numbOfPosit)
                Positions.RemoveAt(0);


            lastPoint = MyMap.Extent.GetCenter();

            if (Positions.Count == numbOfPosit)
            {
                distanceX = (Positions[Positions.Count - 2].X - Positions[Positions.Count - 1].X) * (MyMap.Resolution / 0.3);
                distanceY = (Positions[Positions.Count - 2].Y - Positions[Positions.Count - 1].Y) * (MyMap.Resolution / 0.3);
            }

            textBox1.Text = distanceX.ToString();
            textBox2.Text = distanceY.ToString();

            currentPoint.X = lastPoint.X + distanceX * 100;
            currentPoint.Y = lastPoint.Y - distanceY * 100;
            currentPoint.SpatialReference = currentPoint.SpatialReference;
            MyMap.PanTo(currentPoint);
        }




        private ImageSource ToBitmap(ColorFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;

            byte[] pixels = new byte[width * height * ((PixelFormats.Bgr32.BitsPerPixel + 7) / 8)];

            if (frame.RawColorImageFormat == ColorImageFormat.Bgra)
            {
                frame.CopyRawFrameDataToArray(pixels);
            }
            else
            {
                frame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
            }

            int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
        }

        private Body FirstTrackedBody(Body[] bodies)
        {
            Body primaryBody = null;
            foreach (Body currentBody in bodies)
            {
                if (currentBody.IsTracked != true)
                {
                    continue;
                }

                if (primaryBody == null)
                    primaryBody = currentBody;

                if (EuclidDistance(primaryBody) > EuclidDistance(primaryBody) && EuclidDistance(currentBody) > 0.5)
                    primaryBody= currentBody;
            }

            if (primaryBody == null)
                return null;
            else
                return primaryBody;
        }
        private static double EuclidDistance(Body body)
        {
            double distance = Math.Sqrt(Math.Pow((body.Joints[JointType.SpineMid].Position.X), 2) + Math.Pow((body.Joints[JointType.SpineMid].Position.Y), 2) + Math.Pow((body.Joints[JointType.SpineMid].Position.Z), 2));
            return distance;
        }

    }
}
