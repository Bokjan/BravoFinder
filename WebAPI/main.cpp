#include <map>
#include <string>
#include <memory>
#include <cstdio>
#include <bfinder/bfinder.h>
#include <scaffold/Logger.hpp>
#include <scaffold/Request.hpp>
#include <scaffold/Response.hpp>
#include <scaffold/Scaffold.hpp>
#include <scaffold/RapidJSON.hpp>
#include <rapidjson/filereadstream.h>
using std::string;
using namespace bf;
using namespace rapidjson;
std::map<string, std::shared_ptr<DataSet>> dss;
static void InitializeBF(void)
{
    auto fp = fopen("config.json", "rb");
    if(fp == nullptr)
        throw std::runtime_error("config.json doesn't exists!");
    char readBuffer[1024];
    FileReadStream frs(fp, readBuffer, sizeof(readBuffer));
    Document d;
    d.ParseStream(frs);
    fclose(fp);
    for(auto &i : d.GetArray())
    {
        std::shared_ptr<DataSet> ds(new DataSet);
        ds->SetDataPath(i["path"].GetString());
        ds->Initialize();
        dss.insert(std::make_pair(i["cycle"].GetString(), ds));
    }
    scaf::accesslog.puts("Initialization complete!\n");
}
int main(int argc, char *argv[])
{
    InitializeBF();
    auto app = Scaffold();
    app.get("/", [](Request &req, Response &res)
    {
        res.sendStatus(403);
    });
    app.get("/route/:cycle", [](Request &req, Response &res)
    {
        StringBuffer s;
        Writer<StringBuffer> w(s);
        std::shared_ptr<DataSet> ds;
        std::shared_ptr<Result> result;
        res.type("json");
        w.StartObject();
        w.Key("status");
        if(dss.find(req.params["cycle"]) == dss.end())
        {
            w.Uint(400);
            w.Key("msg");
            w.String("Cycle not available");
            goto ERROR;
        }
        ds = dss[req.params["cycle"]];
        try
        {
            result = ds->FindRoute(req.query["dep"], req.query["arr"]);
        }
        catch(std::exception e)
        {
            w.Uint(500);
            w.Key("msg");
            w.String("cannot calculate a route");
            goto ERROR;
        }
        w.Uint(200);
        w.Key("route");
        w.String(result->route.c_str());
        w.Key("legs");
        w.StartArray();
        for(auto &i : result->legs)
        {
            w.StartObject();
            w.Key("from");
            w.String(i.from.c_str());
            w.Key("to");
            w.String(i.to.c_str());
            w.Key("route");
            w.String(i.route.c_str());
            w.EndObject();
        }
        w.EndArray();
        w.Key("waypoints");
        w.StartArray();
        for(auto &i : result->waypoints)
        {
            w.StartObject();
            w.Key("wpt");
            w.String(i.name.c_str());
            w.Key("lat");
            w.Double(i.coord.latitude);
            w.Key("lon");
            w.Double(i.coord.longitude);
            w.EndObject();
        }
        w.EndArray();
    ERROR:
        w.EndObject();
	string buf = s.GetString();
        res.send(buf.c_str());
    });
    app.listen(3001);
}
