
/*

　山彦セミナー(点群処理：Point Cloud Library)用　サンプルプログラム
　2013.4.3　MTM 作成

　---
　URGデータを読み込んでPCLのPointCloudに保存し、Viewerで表示するプログラムです。
　実行には
　・Point Cloud Library
　・scip2awd
　の2つがインストールされている必要があります。
　また、コンパイルにはcmakeが必要です。

*/


//--------------------------------------------
//include
//--------------------------------------------
#include<cstdio> //stdio.hのc++版
#include<cmath> //math.hのc++版
#include<csignal> //signal.hのc++版
//PCL
#include<pcl/point_types.h> //pcl::PointXYZクラスを使う
#include<pcl/point_cloud.h> //pcl::PointCloudクラスを使う
#include<pcl/visualization/cloud_viewer.h> //cloudViewerクラスを使う
//RPP
#include"scip2awd.h" //ロボ研のURGドライバ(NewPCでインストール済み)




#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//--------------------------------------------
//ctrl-cで停止させるための記述
//--------------------------------------------
bool gIsShuttingDown = false;

void ctrl_c( int aStatus )
{
    gIsShuttingDown = true;
    signal(SIGINT, NULL);
}

//--------------------------------------------
//本体
//--------------------------------------------
int main( int argc , char **argv )
{
    signal(SIGINT, ctrl_c); //ctrl-c停止の設定

    //URG関係の変数宣言
    S2Port      *urgPort; //デバイスファイル名
    S2Sdd_t   urgBuf;   //データを確保するバッファ
    S2Scan_t  *urgData; //バッファへのポインタ
    S2Param_t   urgParam; //URGのパラメータを確保

    //+++++++
    //URGポートのオープン
    //+++++++
    if( argc >= 2 ) //実行時に引数が入力されている場合は、最初の引数のデバイス名を指定してURGをオープンする　そうでなければデフォルト
    {
        urgPort = Scip2_Open( argv[1], B115200 ); //デバイス名,ボーレート設定を指定してオープン
    }
    else
    {
        urgPort = Scip2_Open( "/dev/ttyACM0", B115200 ); //デバイス名(デフォルト),ボーレート設定を指定してオープン
    }

    if( urgPort == 0 ) //ポートが開けなかった場合はエラーメッセージを表示して終了
    {
        printf("[ERROR] Failed to Open Device! \n");
        return 0;
    }
    sleep(1);
    printf("Device Opened.\n");

    S2Sdd_Init( &urgBuf );  //バッファの初期化
    printf("Buffer initialized.\n");

    Scip2CMD_PP(urgPort, &urgParam); //URGパラメータの読み出し
    Scip2CMD_StartMS( urgPort, urgParam.step_min, urgParam.step_max, 1, 0, 0, &urgBuf, SCIP2_ENC_3BYTE ); //垂れ流しモードの開始

    //+++++++
    //PCL関係変数の宣言
    //+++++++
    //PointCloudの宣言
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    //CloudViewerの宣言
    //pcl::visualization::CloudViewer viewer("PCL URG Viewer"); //クラウド表示のためのクラスを定義
  pcl::visualization::CloudViewer viewer("cluster viewer");
    //+++++++
    //メインループ
    //+++++++
    printf("--START--\n");
    while( !gIsShuttingDown ) //ctrl-cかviewerが閉じられるまで繰り返す
  {
    //puts("1");
      int ret = S2Sdd_Begin(&urgBuf, &urgData); //測位データの取り出し
    
        if( ret > 0 )
        {
            cloud2->clear();  //ポイントクラウドの中身を全部クリアする

            for(unsigned int i=0; i < urgData->size; ++i) //URGから取得したすべての点について行う
            {
              if((i>281)&&(i<427)&&(urgData->data[i]<900)){
                double rad = ((double)urgParam.step_min + i - urgParam.step_front) * M_PI * 2 / urgParam.step_resolution; //今のステップの角度を計算
                
                pcl::PointXYZ p;  //１つの点pを定義
                p.x = (double)urgData->data[i] * sin(rad) / 1000; //点pのX座標をURGのデータから計算(1000で割ってミリ表記からメートル表記に直す)
                p.y = (double)urgData->data[i] * cos(rad) / 1000; //点pのY座標をURGのデータから計算(1000で割ってミリ表記からメートル表記に直す)
                p.z = 0.0; //点pのZ座標を計算(常に0としておく)

                cloud2->points.push_back( p ); //作成した点pをcloudに追加する
                }
            }

        S2Sdd_End(&urgBuf); //バッファのアンロック(読み込み終了)
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (5);
        ec.setMaxClusterSize (25000);
        //ec.setSearchMethod (tree);
        ec.setInputCloud( cloud2);
        ec.extract (cluster_indices);

        int j = 0,cou=0;
        float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud2, *cloud_cluster);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
          cloud_cluster->points[*pit].r = colors[j%6][0];
          cloud_cluster->points[*pit].g = colors[j%6][1];
          cloud_cluster->points[*pit].b = colors[j%6][2];
          cou++;
            }
            std::cout << " point count: " << cou << " data points." << std::endl;
            std::cout << "PointCloud representing the Cluster: " << j << " data points." << std::endl;
            std::stringstream ss;
            //ss << "cloud_cluster_" << j << ".pcd";
            //writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
            j++;
        }
        viewer.showCloud (cloud_cluster); 


        }

        else if( ret == -1 ) //戻り値が-1：エラー
        {
            printf("[ERROR] Fatal Error Occured.\n");
        }

        else //戻り値が0：書き込みデータがロック中
        {
            usleep(100);
        }

    }
    printf("--STOP--\n");

    //+++++++
    //終了処理
    //+++++++
    if( Scip2CMD_StopMS( urgPort, &urgBuf ) == 0 ) //URGの測位停止
    {
            printf( "[ERROR] StopMS failed.\n" );
            return 0;
    }
    printf("URG Stopped.\n");

    S2Sdd_Dest( &urgBuf ); //バッファの開放
    printf("Buffer destructed.\n");

    Scip2_Close( urgPort ); //ポートのクローズ
    printf("Port closed.\n");

    return 0;
}

