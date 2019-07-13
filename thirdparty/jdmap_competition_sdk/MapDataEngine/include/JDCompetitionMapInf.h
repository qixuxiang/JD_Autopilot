#pragma once

#include "export.h"
#include  <string>
#include  <vector>
#include <functional>
#include <string>
#include <vector>
#include <memory>
#include <map>

using namespace std;
class MAPDATAENGINE_API CacheResult;
#ifndef _IN_
#define _IN_
#endif

#ifndef _OUT_
#define _OUT_
#endif

namespace JMap {
    /**********************************************************************************************//**
     * \enum  TileType
     *
     * \brief Information about the tile type
     *
     * \author
     * \date  2017/11/10
     **************************************************************************************************/
    enum TileType
    {
        TT_BINARY = 0,         /*用户自定义格式*/
        TT_LAS,                /*LAS格式*/
        TT_PCD,                /*PCD格式*/
        TT_IMAGE_POSITION,     /*定位底图格式，png栅格图片*/
        TT_IMAGE_NAV,          /*导航底图格式，png栅格图片*/
        TT_VECTOR,             /*矢量数据*/
    };

    /**********************************************************************************************//**
     * \struct  TileListRequestParam
     *
     * \brief Information about the param for request tile list.
     *
     * \author
     * \date  2017/11/10
     **************************************************************************************************/
    struct TileListRequestParam
    {
        double lon;                /*经度(wgs84)*/
        double lat;                /*纬度(wgs84)*/
        double radius;             /*半径(单位米)*/
        int level;                 /*层级:层级越大,单个切片范围越小,输入范围[0,31]*/
        TileType type;             /*切片(瓦片)类型*/

        TileListRequestParam()
        {
            lon = 0;
            lat = 0;
            radius = 0;
            level = 19;
            type = TT_PCD;
        }

        TileListRequestParam(const double lo, const  double la,
                const double rad, const int le, const TileType t)
        {
            lon = lo;
            lat = la;
            radius = rad;
            level = le;
            type = t;
        }
    };

    /**********************************************************************************************//**
     * \struct  TileInfoParam
     *
     * \brief Information about the param for request a tile info.
     *
     * \author
     * \date  2017/11/10
     **************************************************************************************************/
    struct TileInfoParam
    {
        std::string strTileName;    /*瓦片名字*/
        TileType type;              /*瓦片类型*/
        double posX;                /*原点X坐标*/
        double posY;                /*原点Y坐标*/
        double posZ;                /*原点Z坐标*/
        double minX;                /*X坐标最小值*/
        double minY;                /*Y坐标最小值*/
        double maxX;                /*X坐标最大值*/
        double maxY;                /*Y坐标最大值*/
        unsigned int crc;           /*CRC校验值*/
        unsigned int version;       /*版本信息*/

        TileInfoParam()
        {
            strTileName = "";
            type = TT_BINARY;
            posX = 0;
            posY = 0;
            posZ = 0;
            minX = 0;
            minY = 0;
            maxX = 0;
            maxY = 0;
            crc = 0;
            version = 0;
        }
    };

    /**********************************************************************************************//**
     * \struct  TileListRequestParam
     *
     * \brief Information about the param for request tile list.
     *
     * \author
     * \date  2017/11/10
     **************************************************************************************************/
    struct TileInfoRequestParam
    {
        std::vector<JMap::TileInfoParam> vTileList;   /*所有瓦片信息*/
        unsigned int rowNum;                          /*请求范围行数*/
        unsigned int colNum;                          /*请求范围列数*/
        double minX;                                  /*X坐标最小值*/
        double minY;                                  /*Y坐标最小值*/

        TileInfoRequestParam()
        {
            rowNum = 0;
            colNum = 0;
            minX = 0;
            minY = 0;
        }
    };

    /**********************************************************************************************//**
     * \struct  TileInfoRequestResult
     *
     * \brief Information about the param for request tile detail information.
     *
     * \author
     * \date  2017/11/10
     **************************************************************************************************/
    struct TileInfoRequestResult
    {
        std::string strName;        /*瓦片名字*/
        char *pBuff;                /*瓦片数据buffer*/
        int len;                    /*瓦片数据长度*/
        TileType type;              /*瓦片类型*/
    };

    /**********************************************************************************************//**
     * \struct  TileErrorCode
     *
     * \brief Information about the error code for request tile.
     *
     * \author
     * \date  2017/11/12
     **************************************************************************************************/
    enum TileErrorCode
    {
        TEC_SUCCESS = 0,                    /*成功*/
        TEC_HTTP_EXCEPTION = 1,             /*HTTP异常,可能端口错误,可能url路径错误*/
        TEC_NET_EXCEPTION = 2,              /*NET异常, 可能没有联网*/
        TEC_TIMEOUT = 3,                    /*超时,可能服务端IP不可访问*/
        TEC_NET_DOWN = 4,                   /*网络不可用, Linux环境下，可能IP不对,端口不对，也有可能网络未连接*/
        TEC_CRC_ERROR = 5,                  /*CRC校验错误*/
        TEC_NO_ROUTE_TO_HOST = 6,           /*No route to host, 可能没有联网*/
        TEC_UNKNOWN = 7,                    /*未知原因*/

        TEC_SERVER_DATA_ERROR = 100,        /*服务端发送的数据有错*/
        TEC_SERVER_FILE_NOT_FOUND = 101,    /*服务端文件没找到*/
    };

    /**********************************************************************************************//**
     * \struct  TileError
     *
     * \brief Information about the error colde for request tile.
     *
     * \author
     * \date  2017/11/12
     **************************************************************************************************/
    struct TileError
    {
        TileErrorCode errCode;        /*错误代码*/
        std::string description;      /*错误描述*/
    };

    /**********************************************************************************************//**
     * \enum  TileInfo
     *
     * \brief Information about the tile detail information.
     *
     * \author
     * \date  2017/11/10
     **************************************************************************************************/
    class MAPDATAENGINE_API TileInfo
    {
        public:
            TileInfo() {}
            TileInfo(const std::string& strName, char*& pBuffer, const int& nLength, const JMap::TileType type, const TileError error, const unsigned int version);
            ~TileInfo();

            const std::string& GetName() const;    /*获得瓦片名字*/
            const char* GetBuffer() const;         /*获得瓦片数据buffer*/
            const int GetLength() const;           /*获得瓦片数据长度*/
            const TileType GetType() const;        /*获得瓦片类型*/
            const TileError& GetError() const;     /*获得错误码*/
            const unsigned int GetVersion() const;  /*获得数据版本号*/
        protected:
            void ReleaseBuffer();

        private:
            std::string m_strName;                /*瓦片名字*/
            char* m_pBuffer;                      /*瓦片数据buffer*/
            int m_nLength;                        /*瓦片数据长度*/
            TileType m_type;                      /*瓦片类型*/
            TileError m_error;                    /*错误码*/
            unsigned int m_version;               /*数据版本号*/
            //void* m_objFactory;
    };

    /**********************************************************************************************//**
     * \struct  ServiceResult
     *
     * \brief Information about service result.
     *
     * \author liaoyi
     * \date  2018/01/23
     **************************************************************************************************/
    struct ServiceResult
    {
        int code;                          /*0 means success, others mean failed*/
        std::string description;           /*error description*/
    };

    class MAPDATAENGINE_API JDMapService
    {
        public:
            /**********************************************************************************************//**
             * \fn virtual JMap::TileInfoRequestParam RequestTilesList(const JMap::TileListRequestParam &region, JMap::TileErrorCode &code) = 0;
             *
             * \brief Request 区域参数范围内的TileList
             *
             * \author  liaoy
             * \date  2017/11/11
             *
             * \param region  区域参数.
             * \param code    错误代码
             **************************************************************************************************/
            virtual JMap::TileInfoRequestParam RequestTilesList(const JMap::TileListRequestParam &region, JMap::TileError &err) = 0;


            /**********************************************************************************************//**
             * \brief 回调函数，在获取到TileList之后，由SDK按照Tile文件，分别进行调用
             *
             * \author  liaoy
             * \date  2017/11/11
             *
             * \param region  Tile文件信息.
             **************************************************************************************************/
            std::function<void(const std::shared_ptr<JMap::TileInfo> param)> onOneTileUpdated;
    };

    /**********************************************************************************************//**
     * \class MAPDATAENGINE_API
     *
     * \brief A mapdataengine api.JMap access point
     *
     * \author  Zjiab
     * \date  2017/5/1
     **************************************************************************************************/
    class MAPDATAENGINE_API ServiceFactory
    {
        public:
            virtual ~ServiceFactory()
            {};

            static ServiceFactory &instance();

            /**********************************************************************************************
             * Creates a service.根据车id和配置创建服务
             *
             * \author zhangjiabin
             * \date 2017/7/19
             *
             * \param ugvId     Identifier for the ugv.
             * \param reservedParam  this param is reserved for further use,ignore it
             *
             * \return null if it fails, else the new service.
             *********************************************************************************************/
            virtual JDMapService *createService(ServiceResult &result) = 0;
    };

}
