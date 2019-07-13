#include "JDCompetitionMapInf.h"
#include <string>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include "assert.h"

#if WIN32
#include <direct.h>
#include <stdio.h>
#include "time.h"
#include <direct.h> 
#include <io.h>
#define SLEEP_1S(s) _sleep(1000*s)
#define g_Mutex        //window下的互斥锁仅为示意，请自行补充完整
#define Lock(x)
#define Unlock(x)
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define SLEEP_1S(s) sleep(s)
#include <cstdio>  
#include <cstdlib>  
#include <unistd.h>  
#include <pthread.h>
#include <sys/types.h>  
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
pthread_mutex_t g_Mutex = PTHREAD_MUTEX_INITIALIZER;
#define Lock(x) pthread_mutex_lock(&x)
#define Unlock(x) pthread_mutex_unlock(&x)
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

int g_CallbackExecutedNum = 0;
int g_TileListNum = 0;
#define TEST_TILE_SAVE_DIR "./CallbackSave_Dir"
/*
*  Tile 更新执行的回调函数，每个Tile都会触发一次调用//
*  该回调函数会在SDK多个线程里执行，所以如果有共享变量，需要由用户层自行进行保护
*  示例仅为Linux环境下的保护措施，请参考如何保护 g_CallbackExecutedNum
*/

void onOneTileUpdated_Callback(const std::shared_ptr<JMap::TileInfo> param);
void TestTileDownload(JMap::JDMapService* service);
int32_t createDirectory(const std::string &directoryPath);

int main()
{
    JMap::ServiceResult sResult;
    JMap::JDMapService *m_service = JMap::ServiceFactory::instance().createService(sResult);

    if (0 == sResult.code)
    {
        TestTileDownload(m_service);
    }

    if (NULL != m_service)
    {
        delete m_service;
        m_service = NULL;
    }
    return 0;
}

void onOneTileUpdated_Callback(const std::shared_ptr<JMap::TileInfo> param)
{
    Lock(g_Mutex);
    g_CallbackExecutedNum++;
    if (g_TileListNum > 0)
    {
        printf("Downloading %s ... %d / %d\n", param->GetName().c_str(), g_CallbackExecutedNum, g_TileListNum);
        fflush(stdout);
    }
    Unlock(g_Mutex);

    if ((param->GetError().errCode == JMap::TEC_SUCCESS) && param->GetBuffer())
    {
        //将Tile保存在当前运行目录的CallbackSave_Dir目录下
        std::string strPath = TEST_TILE_SAVE_DIR;
        std::string strName = strPath;
#if WIN32
        strName.append("\\");
#else
        strName.append("\/");
#endif
        strName.append(param->GetName());

        std::ofstream of(strName, std::ios::binary);
        of.write(param->GetBuffer(), param->GetLength());
        of.close();
    }
    else
    {
        //SDK没有取到具体数据, 由用户层(SDK使用者)根据错误码决定如何处理
        JMap::TileErrorCode errCode = param->GetError().errCode;
        printf("onOneTileUpdated_Callback errorCode=%d\n", (int)errCode);
    }
}

void TestTileDownload(JMap::JDMapService* service)
{
    //设置回调函数
    service->onOneTileUpdated = onOneTileUpdated_Callback;

    std::string save_dir = TEST_TILE_SAVE_DIR;
    if (ACCESS(TEST_TILE_SAVE_DIR, 0) != 0)
    {
        int ret = MKDIR(TEST_TILE_SAVE_DIR);
        if (ret != 0)
        {
            std::cout << "makedir " << std::string(TEST_TILE_SAVE_DIR) << " failed." << std::endl;
        }
        else
        {
            std::cout << "makedir " << std::string(TEST_TILE_SAVE_DIR) << " succeed." << std::endl;
        }
    }

    JMap::TileListRequestParam p;
    JMap::TileError err;
    p.lon = 116.49891977436;
    p.lat = 39.7936580439358;
    p.radius = 500;

    JMap::TileInfoRequestParam result = service->RequestTilesList(p, err);
    if (err.errCode != JMap::TEC_SUCCESS)
    {
        printf("RequestTilesList errorCode=%d\n", (int)err.errCode);
        return;
    }
    Lock(g_Mutex);
    g_TileListNum = result.vTileList.size();
    Unlock(g_Mutex);

    std::cout << "result.vTileList.size=" << result.vTileList.size() << std::endl;

    while(true)
    {
        SLEEP_1S(1);  //等待任务执行结束，然后退出
        if(g_CallbackExecutedNum >= result.vTileList.size())
        {
            std::cout << "Download completed, data file is saved in " << TEST_TILE_SAVE_DIR << std::endl;
            break;
        }
    }
}
