#include <INetworkInteraction.h>
#include <fstream>
#include <yarp/os/LogStream.h>

class FakeNetworkInteraction : public INetworkInteraction
{
    public:
        FakeNetworkInteraction() {};
        std::vector<std::string> getCurrentApName()
        {
            return m_data;
        };
        void loadIwConfigDump(const std::string& file_path)
        {
            std::vector<std::string> data;
            
            // Read the file and save it to data
            std::ifstream file(file_path);
            
            std::string line;
            while (std::getline(file, line))
            {
                data.push_back(line);
            }
            
            m_data = data;
        }
        bool roam(const std::string& ap_name) {
            yInfo() << "Pretending to roam to " << ap_name;
            return true;
        }

    private:
        std::vector<std::string> m_data;
};