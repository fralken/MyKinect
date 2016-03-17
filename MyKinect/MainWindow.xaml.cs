using System;
using System.Linq;
using System.ComponentModel;
using System.Windows;
using System.Windows.Media;

namespace MyKinect
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Kinect manager
        /// </summary>
        private KinectManager _kinect = null;
        private DateTime _startTime;

        private int _fps = 0;
        private double _min_fps = double.MaxValue;
        private double _max_fps = 0.0;
        private double _elapsed_seconds = 0;
        private double _total_frames = 0;

        public MainWindow()
        {
            _kinect = new KinectManager();
            _kinect.GuestHandlerBefore = FPS_counter_starter;
            _kinect.GuestHandlerAfter = UpdateBitmap;

            DataContext = this;

            InitializeComponent();
        }

        private void FPS_counter_starter(object sender, EventArgs e)
        {
            _startTime = DateTime.Now;
            _kinect.GuestHandlerBefore = FPS_counter;
        }

        private void FPS_counter(object sender, EventArgs e)
        {
            TimeSpan elapsed = DateTime.Now - _startTime;
            if (elapsed.TotalMilliseconds >= 1000)
            {
                double current_fps = _fps * 1000 / elapsed.TotalMilliseconds;

                _elapsed_seconds += elapsed.TotalMilliseconds / 1000;
                _total_frames += _fps;
                _fps = 0;
                _startTime = DateTime.Now;

                if (current_fps < _min_fps) _min_fps = current_fps;
                if (current_fps > _max_fps) _max_fps = current_fps;

                double avg_fps = _total_frames / _elapsed_seconds;

                fps.Text = String.Format("FPS: {0:0.00}  min: {1:0.00}  max: {2:0.00}  avg: {3:0.00}", current_fps, _min_fps, _max_fps, avg_fps);
            }
            _fps++;
        }

        private void UpdateBitmap(object sender, EventArgs e)
        {
            if (ImageSource == null)
                ImageSource = _kinect.getBitmap();
        }

        /// <summary>
        /// The image to display.
        /// </summary>
        public ImageSource ImageSource
        {
            get { return (ImageSource)GetValue(ImageProperty); }
            set { SetValue(ImageProperty, value); }
        }
        /// <summary>
        /// The <see cref="ImageSource"/> dependency property.
        /// </summary>
        public static readonly DependencyProperty ImageProperty =
            DependencyProperty.Register("ImageSource", typeof(ImageSource), typeof(MainWindow), new FrameworkPropertyMetadata(null));

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource DrawImage
        {
            get
            {
                //return this.bitmap;
                return _kinect.getDrawImage();
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (_kinect != null)
            {
                Console.WriteLine("*** Dispose Kinect");
                _kinect.Dispose();
                _kinect = null;
            }
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            Console.WriteLine("*** Window Loaded");
            this.MaximizeToSecondaryMonitor();
        }
    }

    static public class WindowExt
    {
        static Point RealPixelsToWpf(Window w, Point p)
        {
            var t = PresentationSource.FromVisual(w).CompositionTarget.TransformFromDevice;
            return t.Transform(p);
        }

        // NB : Best to call this function from the windows Loaded event or after showing the window
        // (otherwise window is just positioned to fill the secondary monitor rather than being maximised).
        public static void MaximizeToSecondaryMonitor(this Window window)
        {
            var secondaryScreen = System.Windows.Forms.Screen.AllScreens.Where(s => !s.Primary).FirstOrDefault();

            Console.WriteLine("*** Screens: {0}", System.Windows.Forms.Screen.AllScreens.Count());
            Console.WriteLine("*** WorkingArea: {0}, {1}, {2}, {3}", window.Left, window.Top, window.Width, window.Height);
            if (secondaryScreen != null)
            {
                if (!window.IsLoaded)
                    window.WindowStartupLocation = WindowStartupLocation.Manual;

                var workingArea = secondaryScreen.WorkingArea;
                var p1 = RealPixelsToWpf(window, new Point(workingArea.Left, workingArea.Top));
                //var p2 = RealPixelsToWpf(window, new Point(workingArea.Width, workingArea.Height));
                window.Left = p1.X;
                window.Top = p1.Y;
                //window.Width = p2.X;
                //window.Height = p2.Y;

                Console.WriteLine("*** WorkingArea: {0}, {1}, {2}, {3}", window.Left, window.Top, window.Width, window.Height);
            }

            // If window isn't loaded then maxmizing will result in the window displaying on the primary monitor
            if (window.IsLoaded)
                window.WindowState = WindowState.Maximized;
            Console.WriteLine("*** WorkingArea: {0}, {1}, {2}, {3}", window.Left, window.Top, window.Width, window.Height);
        }
    }
}
