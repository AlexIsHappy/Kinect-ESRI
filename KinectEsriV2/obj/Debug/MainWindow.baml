   M S B A M L   `   `   `  ����e  aESRI.ArcGIS.Client.Behaviors, Version=2.4.0.851, Culture=neutral, PublicKeyToken=29c6dd6e8553d944qPclr-namespace:ESRI.ArcGIS.Client.Behaviors;assembly=ESRI.ArcGIS.Client.BehaviorsESRI.ArcGIS.Client.Behaviors  5   	    KinectEsriV2   KinectEsriV2.MainWindow   5   	   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   d `Microsoft.Kinect.Wpf.Controls, Version=2.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e350k(http://schemas.microsoft.com/kinect/2014  5   	   :d2http://schemas.microsoft.com/expression/blend/2008  5   	   Dmc;http://schemas.openxmlformats.org/markup-compatibility/2006  5   	   desriBehaviorsPclr-namespace:ESRI.ArcGIS.Client.Behaviors;assembly=ESRI.ArcGIS.Client.Behaviors  5   	   c _ESRI.ArcGIS.Client.Toolkit, Version=2.4.0.851, Culture=neutral, PublicKeyToken=29c6dd6e8553d944o kESRI.ArcGIS.Client.Toolkit.DataSources, Version=2.4.0.851, Culture=neutral, PublicKeyToken=29c6dd6e8553d944Z	 VESRI.ArcGIS.Client, Version=10.2.5.0, Culture=neutral, PublicKeyToken=29c6dd6e8553d944;esri*http://schemas.esri.com/arcgis/client/2009     	 5   	   -   5        $��ControlsBasicsWindow��5	   	     � Title$  
MainWindow��5
   	   $	��768��6   $
��1024��6)    � WindowStyle$
 None=�66    � WindowState$ 	Maximized=�6I    � 
ResizeMode$ NoResize=�5   	    � WindowStartupLocation$ CenterScreen=�6!    � UseLayoutRounding . $��Segoe UI)�5   	   .��5      0  *Microsoft.Kinect.Wpf.Controls.KinectRegion  -   5          Name$ kinectRegion��5      .��5   
   � -   5        .+�5       	 ESRI.ArcGIS.Client.Map  -   5          Name$ MyMap��5      
����ح�$
��Auto��6=      
ZoomFactor$ 1Q�6K   	   MaximumResolution$
	 0.24Q�6Z   
   
WrapAround
 . $��	0,0,0,118q�6�      Extent3 	 -ESRI.ArcGIS.Client.Geometry.EnvelopeConverter$ 
0,-90,0,90 6�   $
��Auto��6�      ZoomDuration$ 	00:00:2.3h�6�      PanDuration$ 00:00:1h�6�      Layers. 5      3 	 -ESRI.ArcGIS.Client.ArcGISTiledMapServiceLayer   	 ESRI.ArcGIS.Client.Layer   Visible . -   5        
LayerNight��	   ID$ 
LayerNight��5   U   
   Url$[ Uhttps://server.arcgisonline.com/arcgis/rest/services/ESRI_Imagery_World_2D/MapServer/��6f     5       .  -   5        StreetLayer��$ StreetLayer��5   W   $\ Vhttps://server.arcgisonline.com/arcgis/rest/services/ESRI_StreetMap_World_2D/MapServer��6h   5      �� 5      -   5         ��Name$ textBox3��5      $y�Visible=�6)   $��55(�6>   $
��Left=�6L   $	��108��6g   $��766,206,0,0q�6t    �� TextWrapping$
 Wrap=�6�   $	��Top=�6�   $	��252��6�   �� 5      -   5        $ textBox1��5      $y�Visible=�6)   $��55(�6>   $
��Left=�6L   $	��108��6g   $��	766,0,0,0q�6t   $
 Wrap=�6�   $	��Top=�6�   $	��252��6�   �� 5      -	   5        $ textBox2��5      $y�Visible=�6)   $��55(�6>   $
��Left=�6L   $��98��6g   $��766,108,0,0q�6s   $
 Wrap=�6�   $	��Top=�6�   $	��252��6�   �� 5      -
   5        $ textBox4��5      $y�Visible=�6)   $��55(�6>   $
��Left=�6L   $	��143��6g   $��766,314,0,0q�6t   $
 Wrap=�6�   $	��Top=�6�   $	��252��6�   �� 5      -   5         ��Name$ scrollViewer��5      $��0,0,-22,118q�61   6H   �� 5      -   5         ��Name$ 
ColorImage��5      $
��Left=�6(   $	��166��6C   $��-15,636,0,-63q�6P   $	��Top=�6g   $	��186��6   5      5      5      