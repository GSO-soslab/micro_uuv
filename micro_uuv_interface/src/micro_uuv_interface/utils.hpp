#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>


namespace soslab {
    std::map<std::string, std::string> parseCommonMoosMsg(const std::string& msg, char delimiter = '=', char separator = ',') {
        std::map<std::string, std::string> result;
        std::istringstream iss(msg);
        std::string word, key, value;
        while (std::getline(iss, word, separator)) {
            std::istringstream sss(word);
            getline(sss, key, delimiter);
            getline(sss, value, delimiter);
            result[key] = value;
        }
        return result;
    }

    std::vector<std::string> parseCommaList(const std::string& msg, char separator = ',') {
        std::string word;
        std::istringstream iss(msg);
        std::vector<std::string> words;
        while(std::getline(iss, word, separator) ){
            words.push_back(word);
        }
        return words;
    }
}