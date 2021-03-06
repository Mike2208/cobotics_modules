<?xml version="1.0" ?>
<sdf version="1.5">
    <model name="avatar_ybot">
        <pose>0.0 0.0 0.0 1.5707963705062866 -0.0 0.0</pose>
        <link name="mixamorig_Hips">
            <pose>0.0 0.996720612049 0.00247051985934 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>5.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.009_convex_hull">
                <pose>0.0 -0.9967206120491028 -0.002470519859343767 -6.077360836798107e-11 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_009_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Surface.009">
                <pose>0.0 -0.9967206120491028 -0.002470519859343767 -6.077360836798107e-11 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_009.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_Spine">
            <pose>1.00000008274e-08 1.09595525265 -0.0098028308712 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>5.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Joints.001_convex_hull">
                <pose>-1.000000082740371e-08 -1.0959553718566895 0.009802831336855888 -6.077360142908717e-11 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_001_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Joints">
                <pose>-1.000000082740371e-08 -1.0959553718566895 0.009802831336855888 -6.077360142908717e-11 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_Spine1">
            <pose>-1.38000019234e-06 1.21240973473 -0.0240262406878 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>5.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Joints.002_convex_hull">
                <pose>1.3799999578623101e-06 -1.2124098539352417 0.024026237428188324 8.705507981154881e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_002_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Surface.010_convex_hull">
                <pose>1.3799998441754724e-06 -1.2124098539352417 0.024026235565543175 8.705507981154881e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_010_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Joints.002">
                <pose>1.3800000715491478e-06 -1.2124098539352417 0.024026239290833473 8.705507981154881e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_002.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Surface.010">
                <pose>1.3799999578623101e-06 -1.2124098539352417 0.024026237428188324 8.705507981154881e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_010.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_Spine2">
            <pose>-4.70000038887e-07 1.34601163865 -0.0402908395044 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>5.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.011_convex_hull">
                <pose>4.699999465174187e-07 -1.346011757850647 0.04029082879424095 2.7331941154784545e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_011_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Surface.011">
                <pose>4.6999997493912815e-07 -1.346011757850647 0.04029083251953125 2.7331941154784545e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_011.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_Neck">
            <pose>-2.20000011096e-07 1.49633657933 -0.0323617900721 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>5.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Joints.003_convex_hull">
                <pose>2.200000039920269e-07 -1.4963366985321045 0.03236179053783417 8.705507981154881e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_003_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Joints.003">
                <pose>2.200000039920269e-07 -1.4963366985321045 0.03236179053783417 8.705507981154881e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_003.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_Head">
            <pose>-1.90000015719e-07 1.59955501557 -0.000937498640196 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>5.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.012_convex_hull">
                <pose>1.8999996598267899e-07 -1.5995547771453857 0.0009374989313073456 -2.564614970043677e-12 -7.105429898699844e-15 -7.105430745732791e-15</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_012_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Surface.012">
                <pose>1.899999801935337e-07 -1.5995548963546753 0.0009374989895150065 -2.564614970043677e-12 -3.552714949349922e-15 -3.5527153728663954e-15</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_012.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_HeadTop_End">
            <pose>1.34999995538e-06 1.78430163861 0.0654264842161 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_LeftEye">
            <pose>0.029478959729 1.67637240887 0.0902698119172 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_RightEye">
            <pose>-0.0294449981404 1.67637240887 0.0902674575337 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_LeftShoulder">
            <pose>0.0610577793245 1.43711578847 -0.0332353296689 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>1.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Joints.008_convex_hull">
                <pose>-0.06105777993798256 -1.437116026878357 0.0332353301346302 8.705507981154881e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_008_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Joints.008">
                <pose>-0.06105777993798256 -1.437116026878357 0.0332353301346302 8.705479670467753e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_008.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_LeftArm">
            <pose>0.187608140839 1.43445670606 -0.0592445502988 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>2.63</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.013_convex_hull">
                <pose>-0.18760810792446136 -1.4344568252563477 0.059244558215141296 -9.920924082251759e-10 7.450583261459087e-09 7.450584149637507e-09</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_013_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Surface.013">
                <pose>-0.18760812282562256 -1.4344568252563477 0.059244554489851 -9.920951837827374e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_013.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftArm_JointLink1">
            <pose>0.187608140839 1.43445670606 -0.0592445502988 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftArm_JointLink2">
            <pose>0.187608140839 1.43445670606 -0.0592445502988 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftForeArm">
            <pose>0.461654979123 1.4344571829 -0.0592444795183 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>1.82</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.014_convex_hull">
                <pose>-0.4616549015045166 -1.4344573020935059 0.05924447998404503 -9.920922972028734e-10 1.4901162970204496e-08 1.4901166522918174e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_014_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Joints.009_convex_hull">
                <pose>-0.4616549015045166 -1.4344573020935059 0.05924447998404503 -9.920922972028734e-10 1.4901162970204496e-08 1.4901166522918174e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_009_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Surface.014">
                <pose>-0.461654931306839 -1.4344573020935059 0.05924447998404503 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_014.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.009">
                <pose>-0.461654931306839 -1.4344573020935059 0.05924447998404503 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_009.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHand">
            <pose>0.737799602879 1.43445742132 -0.0592442820779 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.2</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.015_convex_hull">
                <pose>-0.7377995848655701 -1.434457778930664 0.05924428999423981 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_015_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.010_convex_hull">
                <pose>-0.7377995848655701 -1.434457778930664 0.05924428999423981 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_010_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.015">
                <pose>-0.7377995848655701 -1.4344576597213745 0.05924428626894951 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_015.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.010">
                <pose>-0.7377995848655701 -1.4344576597213745 0.05924428626894951 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_010.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
	<link name="mixamorig_LeftHand_JointLink1">
	    <pose>0.737799602879 1.43445742132 -0.0592442820779 0.0 -0.0 0.0</pose>
	    <inertial>
                <mass>0.2</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
	    <gravity>1</gravity>
	</link>
	<link name="mixamorig_LeftHand_JointLink2">
	    <pose>0.737799602879 1.43445742132 -0.0592442820779 0.0 -0.0 0.0</pose>
	    <inertial>
                <mass>0.2</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
	    <gravity>1</gravity>
	</link>
        <link name="mixamorig_LeftHandMiddle1">
            <pose>0.865554946793 1.43445765974 -0.0592440809122 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.020_convex_hull">
                <pose>-0.8655548095703125 -1.4344576597213745 0.059244073927402496 -9.920896326676143e-10 2.980233304583635e-08 2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_020_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.017_convex_hull">
                <pose>-0.8655548095703125 -1.4344576597213745 0.059244073927402496 -9.920896326676143e-10 2.980233304583635e-08 2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_017_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.020">
                <pose>-0.8655548691749573 -1.434457778930664 0.059244077652692795 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_020.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.017">
                <pose>-0.8655548691749573 -1.434457778930664 0.059244077652692795 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_017.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandMiddle2">
            <pose>0.901694554223 1.43445765974 -0.0592440622857 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.025_convex_hull">
                <pose>-0.9016942977905273 -1.4344578981399536 0.0592440627515316 -9.92092186180571e-10 2.980232594040899e-08 2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_025_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.018_convex_hull">
                <pose>-0.9016942977905273 -1.4344578981399536 0.0592440627515316 -9.92092186180571e-10 2.980232594040899e-08 2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_018_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.025">
                <pose>-0.9016944169998169 -1.4344578981399536 0.0592440627515316 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_025.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.018">
                <pose>-0.9016944169998169 -1.4344578981399536 0.0592440627515316 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_018.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandMiddle3">
            <pose>0.936292129888 1.43445765974 -0.0592443156054 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.028_convex_hull">
                <pose>-0.9362919926643372 -1.4344578981399536 0.0592443086206913 -9.920949617381325e-10 -8.881785255792436e-16 8.881788432165989e-16</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_028_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.019_convex_hull">
                <pose>-0.9362919926643372 -1.4344578981399536 0.0592443086206913 -9.920949617381325e-10 -8.881785255792436e-16 8.881788432165989e-16</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_019_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.028">
                <pose>-0.9362920522689819 -1.4344578981399536 0.0592443123459816 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_028.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.019">
                <pose>-0.9362920522689819 -1.4344578981399536 0.0592443123459816 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_019.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandMiddle4">
            <pose>0.973094004526 1.43445765974 -0.0592439877799 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandThumb1">
            <pose>0.775687772168 1.41278755667 -0.0292134019545 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.016_convex_hull">
                <pose>-0.7756877541542053 -1.4127877950668335 0.029213402420282364 8.705535181618984e-10 2.980233304583635e-08 2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_016_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.013_convex_hull">
                <pose>-0.7756877541542053 -1.4127877950668335 0.029213402420282364 8.705535181618984e-10 2.980233304583635e-08 2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_013_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.016">
                <pose>-0.7756877541542053 -1.4127877950668335 0.029213402420282364 8.705478560244728e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_016.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.013">
                <pose>-0.7756877541542053 -1.4127877950668335 0.029213402420282364 8.705478560244728e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_013.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
	<link name="mixamorig_LeftHandThumb1_JointLink1">
            <pose>0.775687772168 1.41278755667 -0.0292134019545 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
	    <gravity>1</gravity>
	</link>
	<link name="mixamorig_LeftHandThumb1_JointLink2">
            <pose>0.775687772168 1.41278755667 -0.0292134019545 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
	    <gravity>1</gravity>
	</link>
        <link name="mixamorig_LeftHandThumb2">
            <pose>0.812442141904 1.39156746866 -0.00799328042182 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.017_convex_hull">
                <pose>-0.8124422430992126 -1.3915677070617676 0.00799328088760376 -6.07708119937378e-11 -2.980232594040899e-08 -2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_017_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.012_convex_hull">
                <pose>-0.8124422430992126 -1.3915677070617676 0.00799328088760376 -6.07708119937378e-11 -2.980232594040899e-08 -2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_012_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.017">
                <pose>-0.8124421834945679 -1.3915677070617676 0.00799328088760376 -6.077358755129936e-11 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_017.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.012">
                <pose>-0.8124421834945679 -1.3915677070617676 0.00799328088760376 -6.077358755129936e-11 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_012.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandThumb3">
            <pose>0.846385735406 1.3719699383 0.0116046504119 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.018_convex_hull">
                <pose>-0.8463857769966125 -1.3719700574874878 -0.01160464994609356 -6.07708119937378e-11 4.198033920488342e-08 -1.0158488805700472e-07</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_018_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.011_convex_hull">
                <pose>-0.8463857769966125 -1.3719700574874878 -0.01160464994609356 -6.07708119937378e-11 4.198033920488342e-08 -1.0158488805700472e-07</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_011_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.018">
                <pose>-0.8463858366012573 -1.3719700574874878 -0.01160464994609356 -6.077358755129936e-11 4.235164736271502e-22 -1.4356523081460182e-07</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_018.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.011">
                <pose>-0.8463858366012573 -1.3719700574874878 -0.01160464994609356 -6.077358755129936e-11 4.235164736271502e-22 -1.4356523081460182e-07</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_011.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandThumb4">
            <pose>0.873179768531 1.3565004999 0.0270738494584 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandIndex1">
            <pose>0.860465723408 1.4321408272 -0.0310237086377 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.019_convex_hull">
                <pose>-0.8604657053947449 -1.4321409463882446 0.03102370910346508 8.705534071395959e-10 -2.980232594040899e-08 -2.980233304583635e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_019_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.016_convex_hull">
                <pose>-0.8604657053947449 -1.4321409463882446 0.03102370910346508 8.705534071395959e-10 -2.980232594040899e-08 -2.980233304583635e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_016_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.019">
                <pose>-0.8604657053947449 -1.4321409463882446 0.03102370910346508 8.705506315820344e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_019.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.016">
                <pose>-0.8604657053947449 -1.4321409463882446 0.03102370910346508 8.705506315820344e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_016.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandIndex2">
            <pose>0.899385470284 1.4321408272 -0.0310233193449 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.026_convex_hull">
                <pose>-0.8993855714797974 -1.4321409463882446 0.031023316085338593 8.705534071395959e-10 -2.980232594040899e-08 -2.980233304583635e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_026_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.015_convex_hull">
                <pose>-0.8993855714797974 -1.4321409463882446 0.031023316085338593 8.705534071395959e-10 -2.980232594040899e-08 -2.980233304583635e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_015_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.026">
                <pose>-0.8993855118751526 -1.4321409463882446 0.031023317947983742 8.705506315820344e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_026.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.015">
                <pose>-0.8993855118751526 -1.4321409463882446 0.031023317947983742 8.705506315820344e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_015.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandIndex3">
            <pose>0.933537083996 1.4321408272 -0.0310234087519 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.027_convex_hull">
                <pose>-0.933536946773529 -1.4321409463882446 0.031023405492305756 8.705506315820344e-10 -8.881785255792436e-16 8.881787373374805e-16</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_027_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.014_convex_hull">
                <pose>-0.933536946773529 -1.4321409463882446 0.031023405492305756 8.705506315820344e-10 -8.881785255792436e-16 8.881787373374805e-16</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_014_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.027">
                <pose>-0.9335370063781738 -1.4321409463882446 0.031023407354950905 8.705506315820344e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_027.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.014">
                <pose>-0.9335370063781738 -1.4321409463882446 0.031023407354950905 8.705506315820344e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_014.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandIndex4">
            <pose>0.964316922558 1.4321408272 -0.0310234981589 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandRing1">
            <pose>0.859269815815 1.43455636503 -0.0814105789176 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.021_convex_hull">
                <pose>-0.8592697978019714 -1.4345563650131226 0.08141057938337326 -9.920896326676143e-10 2.980233304583635e-08 2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_021_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.020_convex_hull">
                <pose>-0.8592697978019714 -1.4345563650131226 0.08141057938337326 -9.920896326676143e-10 2.980233304583635e-08 2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_020_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.021">
                <pose>-0.8592697978019714 -1.434556484222412 0.08141057938337326 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_021.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.020">
                <pose>-0.8592697978019714 -1.434556484222412 0.08141057938337326 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_020.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandRing2">
            <pose>0.895281511677 1.43455648424 -0.0814102138392 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.024_convex_hull">
                <pose>-0.8952814936637878 -1.4345567226409912 0.0814102366566658 -1.2167959084763424e-08 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_024_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.021_convex_hull">
                <pose>-0.8952814936637878 -1.4345567226409912 0.0814102366566658 -1.2167959084763424e-08 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_021_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.024">
                <pose>-0.8952814936637878 -1.4345567226409912 0.0814102292060852 -1.2167959084763424e-08 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_024.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.021">
                <pose>-0.8952814936637878 -1.4345567226409912 0.0814102292060852 -1.2167959084763424e-08 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_021.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandRing3">
            <pose>0.928354698551 1.43455648363 -0.081410440594 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.029_convex_hull">
                <pose>-0.9283546805381775 -1.4345567226409912 0.08141043782234192 -9.920949617381325e-10 -1.7763570511584873e-15 1.7763576864331977e-15</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_029_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.022_convex_hull">
                <pose>-0.9283546805381775 -1.4345567226409912 0.08141043782234192 -9.920949617381325e-10 -1.7763570511584873e-15 1.7763576864331977e-15</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_022_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.029">
                <pose>-0.9283546805381775 -1.4345567226409912 0.08141043782234192 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_029.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.022">
                <pose>-0.9283546805381775 -1.4345567226409912 0.08141043782234192 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_022.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandRing4">
            <pose>0.96495588435 1.43455648363 -0.081410440594 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandPinky1">
            <pose>0.84688164605 1.43219375612 -0.106502450537 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.022_convex_hull">
                <pose>-0.846881628036499 -1.4321938753128052 0.10650245100259781 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_022_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.023_convex_hull">
                <pose>-0.846881628036499 -1.4321938753128052 0.10650245100259781 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_023_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.022">
                <pose>-0.846881628036499 -1.4321938753128052 0.10650245100259781 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_022.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.023">
                <pose>-0.846881628036499 -1.4321938753128052 0.10650245100259781 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_023.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandPinky2">
            <pose>0.888248163594 1.43219399454 -0.106502189767 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.023_convex_hull">
                <pose>-0.8882481455802917 -1.4321939945220947 0.10650216788053513 1.3909060569972098e-08 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_023_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.024_convex_hull">
                <pose>-0.8882481455802917 -1.4321939945220947 0.10650216788053513 1.3909060569972098e-08 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_024_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.023">
                <pose>-0.8882481455802917 -1.4321941137313843 0.10650217533111572 1.3909059681793678e-08 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_023.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.024">
                <pose>-0.8882481455802917 -1.4321941137313843 0.10650217533111572 1.3909059681793678e-08 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_024.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandPinky3">
            <pose>0.914196449651 1.43219363771 -0.106502544175 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.030_convex_hull">
                <pose>-0.9141964316368103 -1.432193636894226 0.10650255531072617 -8.442678556264127e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_030_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.025_convex_hull">
                <pose>-0.9141964316368103 -1.432193636894226 0.10650255531072617 -8.442678556264127e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_025_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="VIS_Alpha_Surface.030">
                <pose>-0.9141964316368103 -1.4321937561035156 0.10650255531072617 -8.442675003550448e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_030.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.025">
                <pose>-0.9141964316368103 -1.4321937561035156 0.10650255531072617 -8.442675003550448e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_025.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_LeftHandPinky4">
            <pose>0.943435150518 1.43219363692 -0.106502361132 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightShoulder">
            <pose>-0.0610574340994 1.43711698056 -0.0332351992838 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>1.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Joints.026_convex_hull">
                <pose>0.061057426035404205 -1.4371172189712524 0.03323519229888916 8.705534071395959e-10 5.551115784870273e-17 -5.551117108359253e-17</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_026_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Joints.026">
                <pose>0.061057429760694504 -1.4371172189712524 0.03323519602417946 8.705534071395959e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_026.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_RightArm">
            <pose>-0.187607840317 1.43445670605 -0.0592441591434 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>2.63</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.031_convex_hull">
                <pose>0.1876078099012375 -1.4344568252563477 0.05924416705965996 -9.920924082251759e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_031_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision> -->
            <visual name="VIS_Alpha_Surface.031">
                <pose>0.18760782480239868 -1.4344568252563477 0.05924416333436966 -9.920951837827374e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_031.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightArm_JointLink1">
            <pose>-0.187607840317 1.43445670605 -0.0592441591434 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightArm_JointLink2">
            <pose>-0.187607840317 1.43445670605 -0.0592441591434 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightForeArm">
            <pose>-0.461654678601 1.43445718289 -0.0592441069893 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>1.82</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.032_convex_hull">
                <pose>0.4616547226905823 -1.4344573020935059 0.05924411490559578 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_032_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Joints.027_convex_hull">
                <pose>0.4616547226905823 -1.4344573020935059 0.05924411490559578 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_027_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision> -->
            <visual name="VIS_Alpha_Surface.032">
                <pose>0.4616546928882599 -1.4344573020935059 0.05924411118030548 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_032.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.027">
                <pose>0.4616546928882599 -1.4344573020935059 0.05924411118030548 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_027.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHand">
            <pose>-0.737799481171 1.4344573021 -0.0592439281754 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.2</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.033_convex_hull">
                <pose>0.737799346446991 -1.434457540512085 0.05924393609166145 -9.920896326676143e-10 -2.980232949312267e-08 -2.9802340151263707e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_033_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.028_convex_hull">
                <pose>0.737799346446991 -1.434457540512085 0.05924393609166145 -9.920896326676143e-10 -2.980232949312267e-08 -2.9802340151263707e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_028_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.033">
                <pose>0.7377994060516357 -1.434457540512085 0.059243932366371155 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_033.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.028">
                <pose>0.7377994060516357 -1.434457540512085 0.059243932366371155 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_028.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
  <link name="mixamorig_RightHandIndex1_JointLink1">
    <pose>-0.860465482491 1.43214070798 -0.0310233677737 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandIndex2_JointLink1">
    <pose>-0.899385169762 1.43214070798 -0.0310229859314 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandIndex3_JointLink1">
    <pose>-0.933536843079 1.43214070798 -0.0310230809263 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandRing1_JointLink1">
    <pose>-0.85926933648 1.4345561266 -0.081410236191 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandRing2_JointLink1">
    <pose>-0.89528127076 1.4345561266 -0.0814098934643 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandRing3_JointLink1">
    <pose>-0.928354338425 1.43455636472 -0.0814100999739 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandMiddle1_JointLink1">
    <pose>-0.865554586666 1.43445742131 -0.0592437493615 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandMiddle2_JointLink1">
    <pose>-0.901694194096 1.4344573021 -0.0592437009327 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandMiddle3_JointLink1">
    <pose>-0.93629188897 1.43445765973 -0.0592439766042 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandPinky1_JointLink1">
    <pose>-0.846881166714 1.43219351769 -0.10650210781 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandPinky2_JointLink1">
    <pose>-0.888247743862 1.4321936369 -0.106501869391 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>
  <link name="mixamorig_RightHandPinky3_JointLink1">
    <pose>-0.914196149128 1.43219351769 -0.106502227019 0.0 -0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    <gravity>1</gravity>
  </link>


	<link name="mixamorig_RightHand_JointLink1">
	    <pose>-0.737799481171 1.4344573021 -0.0592439281754 0.0 -0.0 0.0</pose>
	    <inertial>
       	        <mass>0.2</mass>
       	        <pose>0 0 0 0 0 0</pose>
       	        <inertia>
       	            <ixx>1.0</ixx>
       	            <ixy>0.0</ixy>
       	            <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
               	    <iyz>0.0</iyz>
           	    <izz>1.0</izz>
            	</inertia>
            </inertial>
	    <gravity>1</gravity>
	</link>
	<link name="mixamorig_RightHand_JointLink2">
	    <pose>-0.737799481171 1.4344573021 -0.0592439281754 0.0 -0.0 0.0</pose>
	    <inertial>
       	        <mass>0.2</mass>
       	        <pose>0 0 0 0 0 0</pose>
       	        <inertia>
       	            <ixx>1.0</ixx>
       	            <ixy>0.0</ixy>
       	            <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
               	    <iyz>0.0</iyz>
           	    <izz>1.0</izz>
            	</inertia>
            </inertial>
	    <gravity>1</gravity>
	</link>
        <link name="mixamorig_RightHandMiddle1">
            <pose>-0.865554586666 1.43445742131 -0.0592437493615 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.038_convex_hull">
                <pose>0.8655545711517334 -1.4344576597213745 0.05924375727772713 -9.920924082251759e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_038_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.035_convex_hull">
                <pose>0.8655545711517334 -1.4344576597213745 0.05924375727772713 -9.920924082251759e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_035_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.038">
                <pose>0.8655545711517334 -1.4344576597213745 0.05924375355243683 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_038.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.035">
                <pose>0.8655545711517334 -1.4344576597213745 0.05924375355243683 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_035.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandMiddle2">
            <pose>-0.901694194096 1.4344573021 -0.0592437009327 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.043_convex_hull">
                <pose>0.9016941785812378 -1.434457778930664 0.05924370139837265 2.733195891835294e-09 -2.980232594040899e-08 -2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_043_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.036_convex_hull">
                <pose>0.9016941785812378 -1.434457778930664 0.05924370139837265 2.733195891835294e-09 -2.980232594040899e-08 -2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_036_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.043">
                <pose>0.9016941785812378 -1.4344576597213745 0.05924370139837265 2.7331932273000348e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_043.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.036">
                <pose>0.9016941785812378 -1.4344576597213745 0.05924370139837265 2.7331932273000348e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_036.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandMiddle3">
            <pose>-0.93629188897 1.43445765973 -0.0592439766042 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.047_convex_hull">
                <pose>0.9362917542457581 -1.4344578981399536 0.05924396961927414 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_047_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.037_convex_hull">
                <pose>0.9362917542457581 -1.4344578981399536 0.05924396961927414 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_037_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.047">
                <pose>0.9362918138504028 -1.4344578981399536 0.05924397334456444 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_047.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.037">
                <pose>0.9362918138504028 -1.4344578981399536 0.05924397334456444 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_037.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandMiddle4">
            <pose>-0.973093763608 1.43445765973 -0.0592436487787 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandThumb1">
            <pose>-0.775687412041 1.41278731824 -0.0292130461894 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.034_convex_hull">
                <pose>0.7756874561309814 -1.4127874374389648 0.029213042929768562 8.705535181618984e-10 -4.198033920488342e-08 1.0158491647871415e-07</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_034_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.031_convex_hull">
                <pose>0.7756874561309814 -1.4127874374389648 0.029213042929768562 8.705535181618984e-10 -4.198033920488342e-08 1.0158491647871415e-07</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_031_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.034">
                <pose>0.7756875157356262 -1.4127874374389648 0.02921304479241371 8.705478560244728e-10 -2.2204500197172533e-16 1.4356525923631125e-07</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_034.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.031">
                <pose>0.7756875157356262 -1.4127874374389648 0.02921304479241371 8.705478560244728e-10 -2.2204500197172533e-16 1.4356525923631125e-07</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_031.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
	<link name="mixamorig_RightHandThumb1_JointLink1">
            <pose>-0.775687412041 1.41278731824 -0.0292130461894 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
	    <gravity>1</gravity>
	</link>
	<link name="mixamorig_RightHandThumb1_JointLink2">
            <pose>-0.775687412041 1.41278731824 -0.0292130461894 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
	    <gravity>1</gravity>
	</link>
        <link name="mixamorig_RightHandThumb2">
            <pose>-0.812441804128 1.3915672321 -0.00799292838201 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Joints.030_convex_hull">
                <pose>0.8124417662620544 -1.3915674686431885 0.007992926985025406 -6.077358755129936e-11 7.105430745732791e-15 -7.1054265105680546e-15</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_030_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Surface.035_convex_hull">
                <pose>0.8124417662620544 -1.3915674686431885 0.007992926985025406 -6.077358755129936e-11 7.105430745732791e-15 -7.1054265105680546e-15</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_035_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision> -->
            <visual name="VIS_Alpha_Joints.030">
                <pose>0.8124417662620544 -1.3915674686431885 0.00799292791634798 -6.077358755129936e-11 -0.0 -1.4210859797399687e-14</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_030.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </visual>
            <visual name="VIS_Alpha_Surface.035">
                <pose>0.8124417662620544 -1.3915674686431885 0.00799292791634798 -6.077358755129936e-11 -0.0 -1.4210859797399687e-14</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_035.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandThumb3">
            <pose>-0.846385457235 1.37196994015 0.011605000589 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.036_convex_hull">
                <pose>0.8463854193687439 -1.3719701766967773 -0.011605000123381615 -6.077358755129936e-11 7.105430745732791e-15 -7.1054265105680546e-15</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_036_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.029_convex_hull">
                <pose>0.8463854193687439 -1.3719701766967773 -0.011605000123381615 -6.077358755129936e-11 7.105430745732791e-15 -7.1054265105680546e-15</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_029_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.036">
                <pose>0.8463854193687439 -1.3719700574874878 -0.011605000123381615 -6.077358755129936e-11 -0.0 -1.4210859797399687e-14</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_036.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.029">
                <pose>0.8463854193687439 -1.3719700574874878 -0.011605000123381615 -6.077358755129936e-11 -0.0 -1.4210859797399687e-14</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_029.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandThumb4">
            <pose>-0.873179473596 1.35650050827 0.0270741977728 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandIndex1">
            <pose>-0.860465482491 1.43214070798 -0.0310233677737 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.037_convex_hull">
                <pose>0.8604653477668762 -1.4321409463882446 0.03102336823940277 8.705535181618984e-10 -2.980233304583635e-08 -2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_037_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.034_convex_hull">
                <pose>0.8604653477668762 -1.4321409463882446 0.03102336823940277 8.705535181618984e-10 -2.980233304583635e-08 -2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_034_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.037">
                <pose>0.860465407371521 -1.4321409463882446 0.03102336823940277 8.705478560244728e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_037.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.034">
                <pose>0.860465407371521 -1.4321409463882446 0.03102336823940277 8.705478560244728e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_034.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandIndex2">
            <pose>-0.899385169762 1.43214070798 -0.0310229859314 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.044_convex_hull">
                <pose>0.8993851542472839 -1.4321411848068237 0.03102298639714718 8.705477450021704e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_044_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.033_convex_hull">
                <pose>0.8993851542472839 -1.4321411848068237 0.03102298639714718 8.705477450021704e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_033_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.044">
                <pose>0.8993851542472839 -1.4321410655975342 0.03102298639714718 8.705477450021704e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_044.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.033">
                <pose>0.8993851542472839 -1.4321410655975342 0.03102298639714718 8.705477450021704e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_033.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandIndex3">
            <pose>-0.933536843079 1.43214070798 -0.0310230809263 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.048_convex_hull">
                <pose>0.9335368275642395 -1.4321411848068237 0.03102308139204979 8.705505205597319e-10 -2.980232594040899e-08 -2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_048_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.032_convex_hull">
                <pose>0.9335368275642395 -1.4321411848068237 0.03102308139204979 8.705505205597319e-10 -2.980232594040899e-08 -2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_032_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.048">
                <pose>0.9335368275642395 -1.4321410655975342 0.03102308139204979 8.705477450021704e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_048.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.032">
                <pose>0.9335368275642395 -1.4321410655975342 0.03102308139204979 8.705477450021704e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_032.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandIndex4">
            <pose>-0.964316681641 1.43214070798 -0.0310231721959 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandRing1">
            <pose>-0.85926933648 1.4345561266 -0.081410236191 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.039_convex_hull">
                <pose>0.8592693209648132 -1.4345561265945435 0.08141022175550461 2.7331965579691087e-09 8.881785255792436e-16 -8.881787373374805e-16</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_039_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.038_convex_hull">
                <pose>0.8592693209648132 -1.4345561265945435 0.08141022175550461 2.7331965579691087e-09 8.881785255792436e-16 -8.881787373374805e-16</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_038_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.039">
                <pose>0.8592693209648132 -1.434556245803833 0.0814102292060852 2.7331936713892446e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_039.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.038">
                <pose>0.8592693209648132 -1.434556245803833 0.0814102292060852 2.7331936713892446e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_038.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandRing2">
            <pose>-0.89528127076 1.4345561266 -0.0814098934643 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.042_convex_hull">
                <pose>0.8952812552452087 -1.4345561265945435 0.08140990138053894 -4.717384260999324e-09 8.881784197001252e-16 -8.881788432165989e-16</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_042_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <!-- <collision name="COL_Alpha_Joints.039_convex_hull">
                <pose>0.8952812552452087 -1.4345561265945435 0.08140990138053894 -4.717384260999324e-09 8.881784197001252e-16 -8.881788432165989e-16</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_039_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.042">
                <pose>0.8952812552452087 -1.434556245803833 0.08140990138053894 -4.717386481445374e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_042.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.039">
                <pose>0.8952812552452087 -1.434556245803833 0.08140990138053894 -4.717386481445374e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_039.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandRing3">
            <pose>-0.928354338425 1.43455636472 -0.0814100999739 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.046_convex_hull">
                <pose>0.9283543229103088 -1.4345566034317017 0.08141012489795685 -4.717380264196436e-09 -2.980232594040899e-08 -2.980233304583635e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_046_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.040_convex_hull">
                <pose>0.9283543229103088 -1.4345566034317017 0.08141012489795685 -4.717380264196436e-09 -2.980232594040899e-08 -2.980233304583635e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_040_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.046">
                <pose>0.9283543229103088 -1.4345566034317017 0.08141011744737625 -4.717380264196436e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_046.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.040">
                <pose>0.9283543229103088 -1.4345566034317017 0.08141011744737625 -4.717380264196436e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_040.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandRing4">
            <pose>-0.964955524224 1.43455636442 -0.0814100904167 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandPinky1">
            <pose>-0.846881166714 1.43219351769 -0.10650210781 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.040_convex_hull">
                <pose>0.8468810319900513 -1.4321935176849365 0.10650209337472916 -9.920924082251759e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_040_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.041_convex_hull">
                <pose>0.8468810319900513 -1.4321935176849365 0.10650209337472916 -9.920924082251759e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_041_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.040">
                <pose>0.846881091594696 -1.432193636894226 0.10650210082530975 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_040.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.041">
                <pose>0.846881091594696 -1.432193636894226 0.10650210082530975 -9.92095072760435e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_041.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandPinky2">
            <pose>-0.888247743862 1.4321936369 -0.106501869391 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.041_convex_hull">
                <pose>0.8882477283477783 -1.432193636894226 0.10650184005498886 6.458488410743257e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_041_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.042_convex_hull">
                <pose>0.8882477283477783 -1.432193636894226 0.10650184005498886 6.458488410743257e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_042_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999998845160007 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.041">
                <pose>0.8882477283477783 -1.4321937561035156 0.10650185495615005 6.458482193494319e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_041.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.042">
                <pose>0.8882477283477783 -1.4321937561035156 0.10650185495615005 6.458482193494319e-09 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_042.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandPinky3">
            <pose>-0.914196149128 1.43219351769 -0.106502227019 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <!-- <collision name="COL_Alpha_Surface.045_convex_hull">
                <pose>0.9141960144042969 -1.4321937561035156 0.10650221258401871 -9.92092186180571e-10 -2.980232594040899e-08 -2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_045_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <collision name="COL_Alpha_Joints.043_convex_hull">
                <pose>0.9141960144042969 -1.4321937561035156 0.10650221258401871 -9.92092186180571e-10 -2.980232594040899e-08 -2.980233659855003e-08</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_043_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>800</mu>
                            <mu2>800</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision> -->
            <visual name="VIS_Alpha_Surface.045">
                <pose>0.9141960740089417 -1.4321937561035156 0.1065022200345993 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_045.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.043">
                <pose>0.9141960740089417 -1.4321937561035156 0.1065022200345993 -9.920949617381325e-10 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_043.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightHandPinky4">
            <pose>-0.943434849995 1.43219351769 -0.106502033304 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>1</gravity>
        </link>
        <link name="mixamorig_RightUpLeg">
            <pose>-0.091244533658 0.930156707764 0.00191673985682 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>7.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.008_convex_hull">
                <pose>0.09124453365802765 -0.9301568865776062 -0.0019167398568242788 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_008_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Joints.050_convex_hull">
                <pose>0.09124453365802765 -0.9301570057868958 -0.0019167398568242788 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_050_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Surface.008">
                <pose>0.09124453365802765 -0.9301568269729614 -0.0019167398568242788 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_008.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.050">
                <pose>0.09124453365802765 -0.930156946182251 -0.0019167398568242788 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_050.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_RightUpLeg_JointLink1">
            <pose>-0.091244533658 0.930156707764 0.00191673985682 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_RightUpLeg_JointLink2">
            <pose>-0.091244533658 0.930156707764 0.00191673985682 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_RightLeg">
            <pose>-0.0936913341284 0.524202227593 -0.0032285398338 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>5.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Joints.046_convex_hull">
                <pose>0.09369133412837982 -0.524202287197113 0.0032285398337990046 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_046_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Surface.005_convex_hull">
                <pose>0.09369133412837982 -0.524202287197113 0.0032285398337990046 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_005_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Joints.046">
                <pose>0.09369133412837982 -0.524202287197113 0.0032285398337990046 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_046.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Surface.005">
                <pose>0.09369133412837982 -0.524202287197113 0.0032285398337990046 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_005.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_RightFoot">
            <pose>-0.0912446454168 0.103723466397 -0.0238308480475 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>1.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Joints.045_convex_hull">
                <pose>0.0912446454167366 -0.10372350364923477 0.023830847814679146 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_045_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Surface.004_convex_hull">
                <pose>0.0912446454167366 -0.10372350364923477 0.023830847814679146 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_004_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Joints.045">
                <pose>0.0912446454167366 -0.10372350364923477 0.023830847814679146 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_045.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Surface.004">
                <pose>0.0912446454167366 -0.10372350364923477 0.023830847814679146 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_004.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_RightToeBase">
            <pose>-0.0949782431126 -0.00119848549299 0.102575791767 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface.003_convex_hull">
                <pose>0.09497824311256409 0.0011984148295596242 -0.10257579386234283 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_003_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Joints.044_convex_hull">
                <pose>0.09497824311256409 0.0011984148295596242 -0.10257579386234283 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_044_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Surface.003">
                <pose>0.09497824311256409 0.0011984166922047734 -0.10257579386234283 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_003.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.044">
                <pose>0.09497824311256409 0.0011984166922047734 -0.10257579386234283 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_044.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_RightToe_End">
            <pose>-0.0949782431126 -0.00119849108093 0.202500996879 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_LeftUpLeg">
            <pose>0.0912445262074 0.930156707764 0.00191673985682 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>7.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Joints.007_convex_hull">
                <pose>-0.09124452620744705 -0.9301568865776062 -0.0019167398568242788 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_007_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Surface.007_convex_hull">
                <pose>-0.09124452620744705 -0.9301568865776062 -0.0019167398568242788 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_007_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Joints.007">
                <pose>-0.09124452620744705 -0.9301568269729614 -0.0019167398568242788 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_007.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Surface.007">
                <pose>-0.09124452620744705 -0.9301568269729614 -0.0019167398568242788 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_007.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_LeftUpLeg_JointLink1">
            <pose>0.0912445262074 0.930156707764 0.00191673985682 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_LeftUpLeg_JointLink2">
            <pose>0.0912445262074 0.930156707764 0.00191673985682 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_LeftLeg">
            <pose>0.0936913266778 0.524202585221 -0.00325366971083 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>5.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Joints.006_convex_hull">
                <pose>-0.09369132667779922 -0.5242026448249817 0.003253669710829854 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_006_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Surface.006_convex_hull">
                <pose>-0.09369132667779922 -0.5242026448249817 0.003253669710829854 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_006_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Joints.006">
                <pose>-0.09369132667779922 -0.5242026448249817 0.003253669710829854 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_006.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Surface.006">
                <pose>-0.09369132667779922 -0.5242026448249817 0.003253669710829854 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_006.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_LeftFoot">
            <pose>0.0912446379662 0.103722512723 -0.0238300797064 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>1.0</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Joints.005_convex_hull">
                <pose>-0.091244637966156 -0.10372254997491837 0.023830080404877663 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_005_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Surface.002_convex_hull">
                <pose>-0.091244637966156 -0.10372254997491837 0.023830080404877663 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_002_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Joints.005">
                <pose>-0.091244637966156 -0.10372254997491837 0.023830080404877663 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_005.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Surface.002">
                <pose>-0.091244637966156 -0.10372254997491837 0.023830080404877663 0.0 -0.0 0.0</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface_002.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_LeftToeBase">
            <pose>0.0949782133103 -0.00119943916699 0.102576545207 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.05</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <collision name="COL_Alpha_Surface_convex_hull">
                <pose>-0.0949782207608223 0.0011993685038760304 -0.10257653892040253 0.0 5.960464477539063e-08 1.6543612251060553e-22</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Surface_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <collision name="COL_Alpha_Joints.004_convex_hull">
                <pose>-0.0949782207608223 0.0011993685038760304 -0.10257653892040253 0.0 5.960464477539063e-08 1.6543612251060553e-22</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/collisions/COL_Alpha_Joints_004_convex_hull.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </collision>
            <visual name="VIS_Alpha_Surface">
                <pose>-0.0949782207608223 0.0011993703665211797 -0.10257653892040253 0.0 5.960464477539063e-08 1.6543612251060553e-22</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Surface.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <visual name="VIS_Alpha_Joints.004">
                <pose>-0.0949782207608223 0.0011993703665211797 -0.10257653892040253 0.0 5.960464477539063e-08 1.6543612251060553e-22</pose>
                <geometry>
                    <mesh>
                        <uri>model://user_avatar_ybot/meshes/visual/VIS_Alpha_Joints_004.dae</uri>
                        <scale>0.009999999776482582 0.009999999776482582 0.009999999776482582</scale>
                    </mesh>
                </geometry>
            </visual>
            <gravity>0</gravity>
        </link>
        <link name="mixamorig_LeftToe_End">
            <pose>0.0949782104792 -0.0011994354417 0.202501742868 0.0 -0.0 0.0</pose>
            <inertial>
                <mass>0.01</mass>
                <pose>0 0 0 0 0 0</pose>
                <inertia>
                    <ixx>1.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>1.0</iyy>
                    <iyz>0.0</iyz>
                    <izz>1.0</izz>
                </inertia>
            </inertial>
            <gravity>0</gravity>
        </link>
        <joint name="mixamorig_Spine" type="fixed">
            <parent>mixamorig_Hips</parent>
            <child>mixamorig_Spine</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-0.17453292519943295</lower>
                    <upper>0.17453292519943295</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_Spine1" type="fixed">
            <parent>mixamorig_Spine</parent>
            <child>mixamorig_Spine1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-0.17453292519943295</lower>
                    <upper>0.17453292519943295</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_Spine2" type="fixed">
            <parent>mixamorig_Spine1</parent>
            <child>mixamorig_Spine2</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-0.17453292519943295</lower>
                    <upper>0.17453292519943295</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_Neck" type="fixed">
            <parent>mixamorig_Spine2</parent>
            <child>mixamorig_Neck</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_Head" type="fixed">
            <parent>mixamorig_Neck</parent>
            <child>mixamorig_Head</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_HeadTop_End" type="fixed">
            <parent>mixamorig_Head</parent>
            <child>mixamorig_HeadTop_End</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftEye" type="fixed">
            <parent>mixamorig_Head</parent>
            <child>mixamorig_LeftEye</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightEye" type="fixed">
            <parent>mixamorig_Head</parent>
            <child>mixamorig_RightEye</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftShoulder" type="fixed">
            <parent>mixamorig_Spine2</parent>
            <child>mixamorig_LeftShoulder</child>
            <axis>
                <xyz>0 1 0</xyz>
                <limit/>
            </axis>
        </joint>
        <joint name="mixamorig_LeftArm_z" type="fixed">
            <parent>mixamorig_LeftShoulder</parent>
            <child>mixamorig_LeftArm_JointLink1</child>
            <axis>
                <xyz>0 0 1</xyz>
                <limit>
                    <effort>1500.0</effort>
                </limit>
                <dynamics>
                    <friction>100.0</friction>
                </dynamics>
            </axis>
        </joint>
        <joint name="mixamorig_LeftArm_x" type="fixed">
            <parent>mixamorig_LeftArm_JointLink1</parent>
            <child>mixamorig_LeftArm_JointLink2</child>
            <axis>
                <xyz>1 0 0</xyz>
                <limit>
                    <effort>1500.0</effort>
                </limit>
                <dynamics>
                    <friction>100.0</friction>
                </dynamics>
            </axis>
        </joint>
        <joint name="mixamorig_LeftArm_y" type="fixed">
            <parent>mixamorig_LeftArm_JointLink2</parent>
            <child>mixamorig_LeftArm</child>
            <axis>
                <xyz>0 1 0</xyz>
                <limit>
                    <effort>1500.0</effort>
                </limit>
                <dynamics>
                    <friction>100.0</friction>
                </dynamics>
            </axis>
        </joint>
        <joint name="mixamorig_LeftForeArm" type="fixed">
            <parent>mixamorig_LeftArm</parent>
            <child>mixamorig_LeftForeArm</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>0</lower>
                    <upper>3.14</upper>
                    <effort>600.0</effort>
                </limit>
                <dynamics>
                    <friction>100.0</friction>
                </dynamics>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHand_z" type="fixed">
            <parent>mixamorig_LeftForeArm</parent>
            <child>mixamorig_LeftHand_JointLink1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <effort>400.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHand_x" type="fixed">
            <parent>mixamorig_LeftHand_JointLink1</parent>
            <child>mixamorig_LeftHand_JointLink2</child>
            <axis>
                <xyz>1.0 0.0 0.0</xyz>
                <limit>
                    <effort>400.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHand_y" type="fixed">
            <parent>mixamorig_LeftHand_JointLink2</parent>
            <child>mixamorig_LeftHand</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <effort>400.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandMiddle1" type="fixed">
            <parent>mixamorig_LeftHand</parent>
            <child>mixamorig_LeftHandMiddle1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandMiddle2" type="fixed">
            <parent>mixamorig_LeftHandMiddle1</parent>
            <child>mixamorig_LeftHandMiddle2</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandMiddle3" type="fixed">
            <parent>mixamorig_LeftHandMiddle2</parent>
            <child>mixamorig_LeftHandMiddle3</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandMiddle4" type="fixed">
            <parent>mixamorig_LeftHandMiddle3</parent>
            <child>mixamorig_LeftHandMiddle4</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandThumb1_z" type="fixed">
            <parent>mixamorig_LeftHand</parent>
            <child>mixamorig_LeftHandThumb1_JointLink1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandThumb1_x" type="fixed">
            <parent>mixamorig_LeftHandThumb1_JointLink1</parent>
            <child>mixamorig_LeftHandThumb1_JointLink2</child>
            <axis>
                <xyz>1.0 0.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandThumb1_y" type="fixed">
            <parent>mixamorig_LeftHandThumb1_JointLink2</parent>
            <child>mixamorig_LeftHandThumb1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandThumb2" type="fixed">
            <parent>mixamorig_LeftHandThumb1</parent>
            <child>mixamorig_LeftHandThumb2</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandThumb3" type="fixed">
            <parent>mixamorig_LeftHandThumb2</parent>
            <child>mixamorig_LeftHandThumb3</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandThumb4" type="fixed">
            <parent>mixamorig_LeftHandThumb3</parent>
            <child>mixamorig_LeftHandThumb4</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandIndex1" type="fixed">
            <parent>mixamorig_LeftHand</parent>
            <child>mixamorig_LeftHandIndex1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandIndex2" type="fixed">
            <parent>mixamorig_LeftHandIndex1</parent>
            <child>mixamorig_LeftHandIndex2</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandIndex3" type="fixed">
            <parent>mixamorig_LeftHandIndex2</parent>
            <child>mixamorig_LeftHandIndex3</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandIndex4" type="fixed">
            <parent>mixamorig_LeftHandIndex3</parent>
            <child>mixamorig_LeftHandIndex4</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandRing1" type="fixed">
            <parent>mixamorig_LeftHand</parent>
            <child>mixamorig_LeftHandRing1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandRing2" type="fixed">
            <parent>mixamorig_LeftHandRing1</parent>
            <child>mixamorig_LeftHandRing2</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandRing3" type="fixed">
            <parent>mixamorig_LeftHandRing2</parent>
            <child>mixamorig_LeftHandRing3</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandRing4" type="fixed">
            <parent>mixamorig_LeftHandRing3</parent>
            <child>mixamorig_LeftHandRing4</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandPinky1" type="fixed">
            <parent>mixamorig_LeftHand</parent>
            <child>mixamorig_LeftHandPinky1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandPinky2" type="fixed">
            <parent>mixamorig_LeftHandPinky1</parent>
            <child>mixamorig_LeftHandPinky2</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandPinky3" type="fixed">
            <parent>mixamorig_LeftHandPinky2</parent>
            <child>mixamorig_LeftHandPinky3</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftHandPinky4" type="fixed">
            <parent>mixamorig_LeftHandPinky3</parent>
            <child>mixamorig_LeftHandPinky4</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightShoulder" type="fixed">
            <parent>mixamorig_Spine2</parent>
            <child>mixamorig_RightShoulder</child>
            <axis>
                <xyz>0 1 0</xyz>
                <limit/>
            </axis>
        </joint>
        <joint name="mixamorig_RightArm_z" type="fixed">
            <parent>mixamorig_RightShoulder</parent>
            <child>mixamorig_RightArm_JointLink1</child>
            <axis>
                <xyz>0 0 1</xyz>
                <limit>
                    <effort>2000.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightArm_x" type="fixed">
            <parent>mixamorig_RightArm_JointLink1</parent>
            <child>mixamorig_RightArm_JointLink2</child>
            <axis>
                <xyz>1 0 0</xyz>
                <limit>
                    <effort>2000.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightArm_y" type="fixed">
            <parent>mixamorig_RightArm_JointLink2</parent>
            <child>mixamorig_RightArm</child>
            <axis>
                <xyz>0 1 0</xyz>
                <limit>
                    <effort>2000.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightForeArm" type="revolute">
            <parent>mixamorig_RightArm</parent>
            <child>mixamorig_RightForeArm</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-3.14</lower>
                    <upper>3.14</upper>
                    <effort>800.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHand_z" type="fixed">
            <parent>mixamorig_RightForeArm</parent>
            <child>mixamorig_RightHand_JointLink1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <effort>400.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHand_x" type="fixed">
            <parent>mixamorig_RightHand_JointLink1</parent>
            <child>mixamorig_RightHand_JointLink2</child>
            <axis>
                <xyz>1.0 0.0 0.0</xyz>
                <limit>
                    <effort>400.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHand_y" type="fixed">
            <parent>mixamorig_RightHand_JointLink2</parent>
            <child>mixamorig_RightHand</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <effort>400.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandMiddle1" type="fixed">
            <parent>mixamorig_RightHand</parent>
            <child>mixamorig_RightHandMiddle1_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandMiddle1_z" type="fixed">
            <parent>mixamorig_RightHandMiddle1_JointLink1</parent>
            <child>mixamorig_RightHandMiddle1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandMiddle2" type="fixed">
            <parent>mixamorig_RightHandMiddle1</parent>
            <child>mixamorig_RightHandMiddle2_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandMiddle2_z" type="fixed">
            <parent>mixamorig_RightHandMiddle2_JointLink1</parent>
            <child>mixamorig_RightHandMiddle2</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandMiddle3" type="fixed">
            <parent>mixamorig_RightHandMiddle2</parent>
            <child>mixamorig_RightHandMiddle3_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandMiddle3_z" type="fixed">
            <parent>mixamorig_RightHandMiddle3_JointLink1</parent>
            <child>mixamorig_RightHandMiddle3</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandMiddle4" type="fixed">
            <parent>mixamorig_RightHandMiddle3</parent>
            <child>mixamorig_RightHandMiddle4</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandThumb1_z" type="fixed">
            <parent>mixamorig_RightHand</parent>
            <child>mixamorig_RightHandThumb1_JointLink1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandThumb1_x" type="fixed">
            <parent>mixamorig_RightHandThumb1_JointLink1</parent>
            <child>mixamorig_RightHandThumb1_JointLink2</child>
            <axis>
                <xyz>1.0 0.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandThumb1_y" type="fixed">
            <parent>mixamorig_RightHandThumb1_JointLink2</parent>
            <child>mixamorig_RightHandThumb1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandThumb2" type="fixed">
            <parent>mixamorig_RightHandThumb1</parent>
            <child>mixamorig_RightHandThumb2</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandThumb3" type="fixed">
            <parent>mixamorig_RightHandThumb2</parent>
            <child>mixamorig_RightHandThumb3</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandThumb4" type="fixed">
            <parent>mixamorig_RightHandThumb3</parent>
            <child>mixamorig_RightHandThumb4</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandIndex1" type="fixed">
            <parent>mixamorig_RightHand</parent>
            <child>mixamorig_RightHandIndex1_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandIndex1_z" type="fixed">
            <parent>mixamorig_RightHandIndex1_JointLink1</parent>
            <child>mixamorig_RightHandIndex1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandIndex2" type="fixed">
            <parent>mixamorig_RightHandIndex1</parent>
            <child>mixamorig_RightHandIndex2_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandIndex2_z" type="fixed">
            <parent>mixamorig_RightHandIndex2_JointLink1</parent>
            <child>mixamorig_RightHandIndex2</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandIndex3" type="fixed">
            <parent>mixamorig_RightHandIndex2</parent>
            <child>mixamorig_RightHandIndex3_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandIndex3_z" type="fixed">
            <parent>mixamorig_RightHandIndex3_JointLink1</parent>
            <child>mixamorig_RightHandIndex3</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandIndex4" type="fixed">
            <parent>mixamorig_RightHandIndex3</parent>
            <child>mixamorig_RightHandIndex4</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandRing1" type="fixed">
            <parent>mixamorig_RightHand</parent>
            <child>mixamorig_RightHandRing1_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandRing1_z" type="fixed">
            <parent>mixamorig_RightHandRing1_JointLink1</parent>
            <child>mixamorig_RightHandRing1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandRing2" type="fixed">
            <parent>mixamorig_RightHandRing1</parent>
            <child>mixamorig_RightHandRing2_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandRing2_z" type="fixed">
            <parent>mixamorig_RightHandRing2_JointLink1</parent>
            <child>mixamorig_RightHandRing2</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandRing3" type="fixed">
            <parent>mixamorig_RightHandRing2</parent>
            <child>mixamorig_RightHandRing3_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandRing3_z" type="fixed">
            <parent>mixamorig_RightHandRing3_JointLink1</parent>
            <child>mixamorig_RightHandRing3</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandRing4" type="fixed">
            <parent>mixamorig_RightHandRing3</parent>
            <child>mixamorig_RightHandRing4</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandPinky1" type="fixed">
            <parent>mixamorig_RightHand</parent>
            <child>mixamorig_RightHandPinky1_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandPinky1_z" type="fixed">
            <parent>mixamorig_RightHandPinky1_JointLink1</parent>
            <child>mixamorig_RightHandPinky1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>

        <joint name="mixamorig_RightHandPinky2" type="fixed">
            <parent>mixamorig_RightHandPinky1</parent>
            <child>mixamorig_RightHandPinky2_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandPinky2_z" type="fixed">
            <parent>mixamorig_RightHandPinky2_JointLink1</parent>
            <child>mixamorig_RightHandPinky2</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandPinky3" type="fixed">
            <parent>mixamorig_RightHandPinky2</parent>
            <child>mixamorig_RightHandPinky3_JointLink1</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandPinky3_z" type="fixed">
            <parent>mixamorig_RightHandPinky3_JointLink1</parent>
            <child>mixamorig_RightHandPinky3</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightHandPinky4" type="fixed">
            <parent>mixamorig_RightHandPinky3</parent>
            <child>mixamorig_RightHandPinky4</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
		    <effort>200.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightUpLeg_z" type="fixed">
            <parent>mixamorig_Hips</parent>
            <child>mixamorig_RightUpLeg_JointLink1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <effort>1500.0</effort>
                </limit>
                <dynamics>
                    <friction>100.0</friction>
                </dynamics>
            </axis>
        </joint>
        <joint name="mixamorig_RightUpLeg_x" type="fixed">
            <parent>mixamorig_RightUpLeg_JointLink1</parent>
            <child>mixamorig_RightUpLeg_JointLink2</child>
            <axis>
                <xyz>1.0 0.0 0.0</xyz>
                <limit>
                    <effort>1500.0</effort>
                </limit>
                <dynamics>
                    <friction>100.0</friction>
                </dynamics>
            </axis>
        </joint>
        <joint name="mixamorig_RightUpLeg_y" type="fixed">
            <parent>mixamorig_RightUpLeg_JointLink2</parent>
            <child>mixamorig_RightUpLeg</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <effort>1500.0</effort>
                </limit>
                <dynamics>
                    <friction>100.0</friction>
                </dynamics>
            </axis>
        </joint>
        <joint name="mixamorig_RightLeg" type="fixed">
            <parent>mixamorig_RightUpLeg</parent>
            <child>mixamorig_RightLeg</child>
            <axis>
                <xyz>1.0 0.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                    <effort>800.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightFoot" type="fixed">
            <parent>mixamorig_RightLeg</parent>
            <child>mixamorig_RightFoot</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit/>
            </axis>
        </joint>
        <joint name="mixamorig_RightToeBase" type="fixed">
            <parent>mixamorig_RightFoot</parent>
            <child>mixamorig_RightToeBase</child>
            <axis>
                <xyz>1.0 0.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_RightToe_End" type="fixed">
            <parent>mixamorig_RightToeBase</parent>
            <child>mixamorig_RightToe_End</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftUpLeg_z" type="fixed">
            <parent>mixamorig_Hips</parent>
            <child>mixamorig_LeftUpLeg_JointLink1</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <effort>800.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftUpLeg_x" type="fixed">
            <parent>mixamorig_LeftUpLeg_JointLink1</parent>
            <child>mixamorig_LeftUpLeg_JointLink2</child>
            <axis>
                <xyz>1.0 0.0 0.0</xyz>
                <limit>
                    <effort>800.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftUpLeg_y" type="fixed">
            <parent>mixamorig_LeftUpLeg_JointLink2</parent>
            <child>mixamorig_LeftUpLeg</child>
            <axis>
                <xyz>0.0 1.0 0.0</xyz>
                <limit>
                    <effort>800.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftLeg" type="fixed">
            <parent>mixamorig_LeftUpLeg</parent>
            <child>mixamorig_LeftLeg</child>
            <axis>
                <xyz>1.0 0.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                    <effort>800.0</effort>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftFoot" type="fixed">
            <parent>mixamorig_LeftLeg</parent>
            <child>mixamorig_LeftFoot</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit/>
            </axis>
        </joint>
        <joint name="mixamorig_LeftToeBase" type="fixed">
            <parent>mixamorig_LeftFoot</parent>
            <child>mixamorig_LeftToeBase</child>
            <axis>
                <xyz>1.0 0.0 0.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                </limit>
            </axis>
        </joint>
        <joint name="mixamorig_LeftToe_End" type="fixed">
            <parent>mixamorig_LeftToeBase</parent>
            <child>mixamorig_LeftToe_End</child>
            <axis>
                <xyz>0.0 0.0 1.0</xyz>
                <limit>
                    <lower>-1.5707963267948966</lower>
                    <upper>1.5707963267948966</upper>
                </limit>
            </axis>
        </joint>
        <!-- <plugin filename="libavatar_control_plugin.so" name="avatar_ybot_controller">
            <controller type="all_joints_pid_position_target"/>
            <controller type="model_pose_target">
                <root_link>mixamorig_Hips</root_link>
            </controller>
            <controller type="constant_joint_forces"/>
        </plugin> -->
         <plugin filename="libavatar_model_grasp.so" name="avatar_model_grasp">
        </plugin>
    </model>
</sdf>
