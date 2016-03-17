using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace MyKinect
{
    class KinectManager
    {
        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private readonly int _bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor _kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper _coordinateMapper = null;

        /// <summary>
        /// Reader for depth/color/body index frames
        /// </summary>
        private MultiSourceFrameReader _multiFrameSourceReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap _bitmap = null;

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup _drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage _drawImage;

        /// <summary>
        /// The size in bytes of the bitmap back buffer
        /// </summary>
        private uint _bitmapBackBufferSize = 0;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int _depthWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int _depthHeight;

        /// <summary>
        /// Width of display (color space)
        /// </summary>
        private int _colorWidth;

        /// <summary>
        /// Height of display (color space)
        /// </summary>
        private int _colorHeight;

        /// <summary>
        /// Intermediate storage for the color to depth mapping
        /// </summary>
        private DepthSpacePoint[] _colorMappedToDepthPoints = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] _bodies = null;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float _InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush _handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush _handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush _handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double _HandSize = 30;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double _ClipBoundsThickness = 10;

        private EventHandler<EventArgs> _guestHandler = null;
             
        public KinectManager()
        {
            _kinectSensor = KinectSensor.GetDefault();

            _multiFrameSourceReader = _kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.BodyIndex | FrameSourceTypes.Body);

            _multiFrameSourceReader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

            _coordinateMapper = _kinectSensor.CoordinateMapper;

            FrameDescription depthFrameDescription = _kinectSensor.DepthFrameSource.FrameDescription;

            _depthWidth = depthFrameDescription.Width;
            _depthHeight = depthFrameDescription.Height;

            FrameDescription colorFrameDescription = _kinectSensor.ColorFrameSource.FrameDescription;

            _colorWidth = colorFrameDescription.Width;
            _colorHeight = colorFrameDescription.Height;

            _colorMappedToDepthPoints = new DepthSpacePoint[_colorWidth * _colorHeight];

            InitializeBitmaps();

            _kinectSensor.Open();
        }

        private void InitializeBitmaps()
        {
            double dpiX = 96.0;
            double dpiY = 96.0;

            _bitmap = new WriteableBitmap(_colorWidth, _colorHeight, dpiX, dpiY, PixelFormats.Bgra32, null);

            // Create the drawing group we'll use for drawing
            _drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            _drawImage = new DrawingImage(_drawingGroup);

            System.Console.WriteLine("*** Depth image pixel width {0:N} pixel height {1:N}", _depthWidth, _depthHeight);
            System.Console.WriteLine("*** Color image pixel width {0:N} pixel height {1:N} dpiX {2} dpiY {3}", _colorWidth, _colorHeight, dpiX, dpiY);
            System.Console.WriteLine("*** Bitmap pixel width {0:N} height {1:N}", _bitmap.PixelWidth, _bitmap.PixelHeight);
            System.Console.WriteLine("*** Bitmap image width {0:N} height {1:N}", _bitmap.Width, _bitmap.Height);
            System.Console.WriteLine("*** Draw image width {0:N} height {1:N}", _drawImage.Width, _drawImage.Height);

            // Calculate the WriteableBitmap back buffer size
            _bitmapBackBufferSize = (uint)((_bitmap.BackBufferStride * (_bitmap.PixelHeight - 1)) + (_bitmap.PixelWidth * _bytesPerPixel));
            System.Console.WriteLine("*** _bitmapBackBufferSize {0}", _bitmapBackBufferSize);
        }

        public ImageSource getBitmap()
        {
            return _bitmap;
        }

        public ImageSource getDrawImage()
        {
            return _drawImage;
        }

        public void add(EventHandler<EventArgs> e)
        {
            _guestHandler = e;
        }

        public void Dispose()
        {
            if (_multiFrameSourceReader != null)
            {
                // MultiSourceFrameReder is IDisposable
                _multiFrameSourceReader.Dispose();
                _multiFrameSourceReader = null;
            }

            if (_kinectSensor != null)
            {
                _kinectSensor.Close();
                _kinectSensor = null;
            }
        }


        /// <summary>
        /// Handles the depth/color/body index frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            if (_guestHandler != null)
                _guestHandler(sender, e);

            int depthWidth = 0;
            int depthHeight = 0;

            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;
            BodyIndexFrame bodyIndexFrame = null;
            BodyFrame bodyFrame = null;
            bool isBitmapLocked = false;

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            // We use a try/finally to ensure that we clean up before we exit the function.  
            // This includes calling Dispose on any Frame objects that we may have and unlocking the bitmap back buffer.
            try
            {
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame();
                bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame();

                // If any frame has expired by the time we process this event, return.
                // The "finally" statement will Dispose any that are not null.
                if ((depthFrame == null) || (colorFrame == null) || (bodyIndexFrame == null) || (bodyFrame == null))
                {
                    return;
                }

                // Process Depth
                FrameDescription depthFrameDescription = depthFrame.FrameDescription;

                depthWidth = depthFrameDescription.Width;
                depthHeight = depthFrameDescription.Height;

                // Access the depth frame data directly via LockImageBuffer to avoid making a copy
                using (KinectBuffer depthFrameData = depthFrame.LockImageBuffer())
                {
                    _coordinateMapper.MapColorFrameToDepthSpaceUsingIntPtr(
                        depthFrameData.UnderlyingBuffer,
                        depthFrameData.Size,
                        _colorMappedToDepthPoints);
                }

                // We're done with the DepthFrame 
                depthFrame.Dispose();
                depthFrame = null;

                // Process Color

                // Lock the bitmap for writing
                _bitmap.Lock();
                isBitmapLocked = true;

                if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                {
                    colorFrame.CopyRawFrameDataToIntPtr(_bitmap.BackBuffer, _bitmapBackBufferSize);
                }
                else
                {
                    colorFrame.CopyConvertedFrameDataToIntPtr(_bitmap.BackBuffer, _bitmapBackBufferSize, ColorImageFormat.Bgra);
                }

                // We're done with the ColorFrame 
                colorFrame.Dispose();
                colorFrame = null;

                // We'll access the body index data directly to avoid a copy
                using (KinectBuffer bodyIndexData = bodyIndexFrame.LockImageBuffer())
                {
                    unsafe
                    {
                        byte* bodyIndexDataPointer = (byte*)bodyIndexData.UnderlyingBuffer;

                        int colorMappedToDepthPointCount = _colorMappedToDepthPoints.Length;

                        fixed (DepthSpacePoint* colorMappedToDepthPointsPointer = _colorMappedToDepthPoints)
                        {
                            // Treat the color data as 4-byte pixels
                            uint* bitmapPixelsPointer = (uint*)_bitmap.BackBuffer;

                            // Loop over each row and column of the color image
                            // Zero out any pixels that don't correspond to a body index
                            for (int colorIndex = 0; colorIndex < colorMappedToDepthPointCount; ++colorIndex)
                            {
                                float colorMappedToDepthX = colorMappedToDepthPointsPointer[colorIndex].X;
                                float colorMappedToDepthY = colorMappedToDepthPointsPointer[colorIndex].Y;

                                // The sentinel value is -inf, -inf, meaning that no depth pixel corresponds to this color pixel.
                                if (!float.IsNegativeInfinity(colorMappedToDepthX) &&
                                    !float.IsNegativeInfinity(colorMappedToDepthY))
                                {
                                    // Make sure the depth pixel maps to a valid point in color space
                                    int depthX = (int)(colorMappedToDepthX + 0.5f);
                                    int depthY = (int)(colorMappedToDepthY + 0.5f);

                                    // If the point is not valid, there is no body index there.
                                    if ((depthX >= 0) && (depthX < depthWidth) && (depthY >= 0) && (depthY < depthHeight))
                                    {
                                        int depthIndex = (depthY * depthWidth) + depthX;

                                        // If we are tracking a body for the current pixel, do not zero out the pixel
                                        if (bodyIndexDataPointer[depthIndex] != 0xff)
                                        {
                                            continue;
                                        }
                                    }
                                }

                                bitmapPixelsPointer[colorIndex] = 0;
                            }
                        }

                        _bitmap.AddDirtyRect(new Int32Rect(0, 0, _bitmap.PixelWidth, _bitmap.PixelHeight));
                    }
                }

                // bodyFrame
                if (_bodies == null)
                {
                    _bodies = new Body[bodyFrame.BodyCount];
                }

                // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                // As long as those body objects are not disposed and not set to null in the array,
                // those body objects will be re-used.
                bodyFrame.GetAndRefreshBodyData(_bodies);
                bodyFrame.Dispose();
                bodyFrame = null;

                using (DrawingContext dc = _drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    //dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, _depthWidth, _depthHeight));
                    dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, _colorWidth, _colorHeight));

                    foreach (Body body in _bodies)
                    {
                        if (body.IsTracked)
                        {
                            DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = _InferredZPositionClamp;
                                }

                                //DepthSpacePoint depthSpacePoint = _coordinateMapper.MapCameraPointToDepthSpace(position);
                                //jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                ColorSpacePoint colorSpacePoint = _coordinateMapper.MapCameraPointToColorSpace(position);
                                jointPoints[jointType] = new Point(colorSpacePoint.X, colorSpacePoint.Y);
                            }

                            DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    //_drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, _depthWidth, _depthHeight));
                    _drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, _colorWidth, _colorHeight));
                }
            }
            finally
            {
                if (isBitmapLocked)
                    _bitmap.Unlock();

                if (depthFrame != null)
                    depthFrame.Dispose();

                if (colorFrame != null)
                    colorFrame.Dispose();

                if (bodyIndexFrame != null)
                    bodyIndexFrame.Dispose();

                if (bodyFrame != null)
                    bodyFrame.Dispose();
            }
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(_handClosedBrush, null, handPosition, _HandSize, _HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(_handOpenBrush, null, handPosition, _HandSize, _HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(_handLassoBrush, null, handPosition, _HandSize, _HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            //int width = _depthWidth;
            //int height = _depthHeight;
            int width = _colorWidth;
            int height = _colorHeight;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, height - _ClipBoundsThickness, width, _ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, width, _ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, _ClipBoundsThickness, height));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(width - _ClipBoundsThickness, 0, _ClipBoundsThickness, height));
            }
        }
    }
}
