//!
//!@file    tsvio.h
//!@author  Alexey Veselovsky
//!@date    15.02.2018
//!@brief   tsv reader & tsv writer
//!

#pragma once

#ifndef AR13_TSVIO_H
#define AR13_TSVIO_H

#include <stdio.h>
#include <cstdint>
#include <string>
//#include <ar10/sx_str.h>
#include <algorithm>
#include <stdlib.h>
#include <vector>
#include <map>
#include <string.h>
#include <limits>

#include <cctype>
#include <locale>

#if defined _MSC_VER && (_MSC_VER == 1700)
  #define PRId64       "lld"
  #define EXPLICIT_OP
#else 
  #include <cinttypes>
  #define EXPLICIT_OP explicit
#endif


#include <typeinfo>

namespace ar {

struct Value {
  enum Type {
    NONE,
    INT64,
    FLOAT64,
    STRING
  };

  inline Value();
  inline Value(int v);
  inline Value(int64_t v);
  inline Value(float v);
  inline Value(double v);
  inline Value(const std::string& v);
  inline Value(const char* str, Value::Type t = NONE);

  inline int64_t asLong(int64_t default_value = INT64_MIN) const;
  inline int64_t asInteger(int64_t default_value = INT64_MIN) const;
  inline double asFloat64(double default_value = std::numeric_limits<double>::quiet_NaN()) const;
  inline double asDouble(double default_value = std::numeric_limits<double>::quiet_NaN()) const;
  inline std::string asString(const std::string& default_value = "") const;

  inline EXPLICIT_OP operator std::string() const;  //!< throws std::bad_cast
  inline EXPLICIT_OP operator int64_t() const;      //!< throws std::bad_cast
  inline EXPLICIT_OP operator double() const;       //!< throws std::bad_cast

  // fields
  std::string str_;
  union {
    int64_t i64;
    double f64;
  } simple_val_;
  Type type_;
};

/** \class TsvReader
 *  \brief Reads typed tsv (tab separated value) file format
 *
 *  typed tsv file contains two rows header:
 *     first row  - column names
 *     second row - column types
 *
 *  example:
 *    column1 column2 column3
 *    long double string
 *
 *  Possible column types:
 *     - string
 *     - long (aka int64_t)
 *     - double (float64)
 *
 *  Usage example:
 *  ar::TsvReader reader;
 *  // open file
 *  auto foo = reader.open("t24.305.025.tsv");
 *  // column types
 *  for (auto t : reader.types_)
 *    std::cout << t << std::endl;
 *  // read all data rows
 *  while (reader.readNextRow()) {
 *      std::cout << reader("grabMsec").asLong()
 *                << " " << reader("some").asLong()
 *                << std::endl;
 *  }
 */

class TsvReader {
public:
  typedef std::map<std::string, Value> Row;

  inline TsvReader();
  inline ~TsvReader();

  inline bool open(const std::string& filepath);
  inline std::string getCommentedHeader() const {return commented_header_;}
  inline bool readNextRow(bool validate = false); //!< reads next row, returns false if fail
  inline Value operator()(const std::string& columnname, bool* pres = nullptr) const; //!< get cell from current row

  Row current_line_;                //!< last read data row
  std::vector<std::string> header_; //!< column names
  std::vector<Value::Type> types_;  //!< column types
  std::string commented_header_;
private:
  bool has_types_header_;
  bool first_row_;
  FILE* in_;
};

class TsvWriter {
public:
  inline TsvWriter() : write_header(true), out(nullptr){}
  inline ~TsvWriter(){close();}

  typedef std::pair<std::string, Value::Type> Field;
  typedef std::vector<Field> Header;

  inline void setHeader(const Header& h);
  inline bool open(const std::string& filename);
  inline void close();
  inline bool writeRow(const std::vector<Value>& row);
private:
  bool write_header;
  Header hdr;
  FILE* out;
};

// Some string utility functions. TODO move to sx_string (ore replace with calls from sx_string)

namespace xtd{
  // trim functionality taken from here: https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring

  // trim from start (in place)
  static inline void ltrim(std::string &s) {
      s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
          return !std::isspace(ch);
      }));
  }

  // trim from end (in place)
  static inline void rtrim(std::string &s) {
      s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
          return !std::isspace(ch);
      }).base(), s.end());
  }

  // trim from both ends (in place)
  static inline void trim(std::string &s) {
      ltrim(s);
      rtrim(s);
  }

  // trim from start (copying)
  static inline std::string ltrim_copy(std::string s) {
      ltrim(s);
      return s;
  }

  // trim from end (copying)
  static inline std::string rtrim_copy(std::string s) {
      rtrim(s);
      return s;
  }

  // trim from both ends (copying)
  static inline std::string trim_copy(std::string s) {
      trim(s);
      return s;
  }
}

// Value implementation
Value::Value() : type_(NONE) {}
Value::Value(int v) : type_(INT64) {simple_val_.i64 = v;}
Value::Value(int64_t v) : type_(INT64) {simple_val_.i64 = v ;}
Value::Value(float v) : type_(FLOAT64) {simple_val_.f64 = v;}
Value::Value(double v) : type_(FLOAT64) {simple_val_.f64 = v;}
Value::Value(const std::string& v) : type_(STRING), str_(v) {}

Value::Value(const char* str, Value::Type t) : str_(str), type_(str_.length() ? STRING : NONE)
{
  //prun: добавил trim первым шагом, чтобы удалять слцчайные разделители.
  xtd::trim(str_);
  bool is_int = std::all_of(str_.begin(), str_.end(), [](const char c){
    return ('0' <= c && c<='9' || c=='-'); //std::isdigit? stoi? 
  });

  bool is_float = std::all_of(str_.begin(), str_.end(), [](const char c){
    return (('0' <= c && c<='9') || c=='e' || c=='E' || c=='.' || c=='-' || c=='+'); 
  });

  if (t != NONE) {

  } else {
    if (is_int) {
      simple_val_.i64 = std::stol(str_);
      type_ = INT64;
    } else if (is_float) {
      try {
        simple_val_.f64 = std::stod(str_);
        type_ = FLOAT64;
      } catch (...) {
      }
    }
  }
}


Value::operator std::string() const
{
  if (type_ != STRING) throw std::bad_cast();
  return str_;
}

Value::operator int64_t() const
{
  if (type_ != INT64) throw std::bad_cast();
  return simple_val_.i64;
}

Value::operator double() const
{
  if (type_ != FLOAT64) throw std::bad_cast();
  return simple_val_.f64;
}


int64_t Value::asLong(int64_t default_value) const
{
  return asInteger(default_value);
}

int64_t Value::asInteger(int64_t default_value) const
{
  int64_t  res = type_ == INT64 ? simple_val_.i64 : default_value;
  return res;
}

double Value::asDouble(double default_value) const
{
  return asFloat64(default_value);
}

double Value::asFloat64(double default_value) const
{
  double  res = type_ == FLOAT64 ? simple_val_.f64 : default_value;
  return res;
}

std::string Value::asString(const std::string& default_value) const
{
  std::string res = type_ == STRING ? str_ : default_value;
  return res;
}


// TsvReader implementation

inline int GetLine(char **lineptr, size_t *n, FILE *fp);
inline char* Xstrtok(char *line, char *delims);

TsvReader::TsvReader() : in_(nullptr), has_types_header_(false), first_row_(false) {}
TsvReader::~TsvReader() {if (in_) fclose(in_);}

bool TsvReader::open(const std::string& filepath)
{
  in_ = fopen(filepath.c_str(), "r");
  if (!in_) return false;
  // read header
  char* line = nullptr;
  size_t n = 0;
  for (;;) {
    int r = GetLine(&line, &n, in_);
    if (r==-1) {return false;}
    if (line[0] == '#') {
      commented_header_.append(line+1);
      //commented_header_.append("\n");
    } else {
      break;
    }
  }
  char* buf = line;
  char* value;
  while (value = strtok(buf, "\t")) {
    buf = nullptr;
    for (char* c=value; *c; c++) {
      if (*c == '\n')
        *c = 0;
    }
    if (value[0] != '\n')
      header_.emplace_back(value);
  }
  free(line);

  // try to read type header
  readNextRow();
  has_types_header_ = std::all_of(current_line_.begin(),current_line_.end(),
                                  [](const std::pair<std::string, Value>& p) {
                                    return p.second.asString() == "int" || p.second.asString() == "long" ||
                                      p.second.asString() == "double" || p.second.asString() == "string";
                                  });
  if (has_types_header_) {
    for (auto i : current_line_) {
      auto t = Value::NONE;
      if (i.second.asString() == "int" || i.second.asString() == "long") t = Value::INT64;
      if (i.second.asString() == "double") t = Value::FLOAT64;
      if (i.second.asString() == "string") t = Value::STRING;
      types_.push_back(t);
    }
  }
  first_row_ = !has_types_header_;
  return in_;
}

bool TsvReader::readNextRow(bool validate)
{
  if (first_row_) {
    first_row_ = false;
    return true;
  }

  current_line_.clear();

  char* line = nullptr;

  {
    size_t n = 0;
    int r = GetLine(&line, &n, in_);
    if (r == -1) {
      fclose(in_);
      in_ = nullptr;
      return false;
    }
  }

  char* buf = line;
  char* value;

  int n = 0;
  while (value = Xstrtok(buf, "\t")) {
    buf = nullptr;
    for (char* c=value; *c; c++) {if (*c == '\n') *c = 0;}
    if (value[0] != '\n' && value[0]!='\0') {
      if (n>header_.size()) {
        free(line);
        return false;
      }
      current_line_[header_[n]] = Value(value);
    }
    n++;
  }
  free(line);
  if (validate) {
    size_t j=0;
    for (auto i : current_line_) {
      if (i.second.type_ != types_[j++])
        return false;
    }
  }
  return true;
}

Value TsvReader::operator()(const std::string& key, bool* pres) const
{
  auto i = current_line_.find(key);
  if (i != current_line_.end()) {
    if (pres) *pres = true;
    return i->second;
  } else {
    if (pres) *pres = false;
    return Value();
  }
}


// TsvWriter implementation

void TsvWriter::setHeader(const Header& h)
{
  hdr = h;
}

bool TsvWriter::open(const std::string& filename)
{
  out = fopen(filename.c_str(), "w+");
  return out!=nullptr;
}

void TsvWriter::close()
{
  if (out) fclose(out);
  out = nullptr;
}

bool TsvWriter::writeRow(const std::vector<Value>& row)
{
  if (!out)
    return false;
  if (hdr.empty())
    return false;
  // write header
  if (write_header) {
    write_header = false;
    // names
    bool prev = false;
    for (const auto& h : hdr) {
      if (prev) fprintf(out, "%s", "\t");
      prev = true;
      fprintf(out, "%s", h.first.c_str());
    }
    fprintf(out, "\n");
    // types
    prev = false;
    for (const auto& h : hdr) {
      if (prev) fprintf(out, "%s", "\t");
      prev = true;
      switch (h.second) {
      case Value::INT64:
        fprintf(out, "%s", "long");
        break;
      case Value::FLOAT64:
        fprintf(out, "%s", "double");
        break;
      case Value::STRING:
        fprintf(out, "%s", "string");
        break;
      case Value::NONE:
        return false;
      }
    }
    fprintf(out, "\n");
  }
  // check sizes
  if (row.size()!=hdr.size())
    return false;
  // check types
  for (size_t i=0; i<hdr.size(); ++i)
    if (hdr[i].second != row[i].type_ && row[i].type_ != Value::NONE)
      return false;
  // finaly let's write the row!
  bool prev = false;
  for (const auto& v : row) {
    if (prev) fprintf(out, "\t");
    prev = true;
    switch(v.type_) {
    case Value::INT64:
      fprintf(out, "%" PRId64, v.asInteger());
      break;
    case Value::FLOAT64:
      fprintf(out, "%lf", v.asFloat64());
      break;
    case Value::STRING:
      fprintf(out, "%s", v.asString().c_str());
      break;
    case Value::NONE:
      break;
    }
  }
  fprintf(out, "\n");
  return true;
}

inline int GetLine(char **lineptr, size_t *n, FILE *fp) {
#define GETLINE_MINSIZE 16
  int ch;
  int i = 0;
  char free_on_err = 0;
  char *p;

  errno = 0;
  if (lineptr == NULL || n == NULL || fp == NULL) {
    errno = EINVAL;
    return -1;
  }
  if (*lineptr == NULL) {
    *n = GETLINE_MINSIZE;
    *lineptr = (char *)malloc( sizeof(char) * (*n));
    if (*lineptr == NULL) {
      errno = ENOMEM;
      return -1;
    }
    free_on_err = 1;
  }

  for (i=0; ; i++) {
    ch = fgetc(fp);
    while (i >= (*n) - 2) {
      *n *= 2;
      p = (char*)realloc(*lineptr, sizeof(char) * (*n));
      if (p == NULL) {
        if (free_on_err)
          free(*lineptr);
        errno = ENOMEM;
        return -1;
      }
      *lineptr = p;
    }
    if (ch == EOF) {
      if (i == 0) {
        if (free_on_err)
          free(*lineptr);
        return -1;
      }
      (*lineptr)[i] = '\0';
      *n = i;
      return i;
    }

    if (ch == '\n') {
      (*lineptr)[i] = '\n';
      (*lineptr)[i+1] = '\0';
      *n = i+1;
      char* r = strchr(*lineptr, '\r');
      if (r) *r = 0;
      return i+1;
    }
    (*lineptr)[i] = (char)ch;
  }
#undef GETLINE_MINSIZE
}

inline char* Xstrtok(char *line, char *delims)
{
  static char *saveline = NULL;
  char *p;
  int n;

  if(line != NULL)
    saveline = line;

  /*
*see if we have reached the end of the line
*/
  if(saveline == NULL || *saveline == '\0')
    return(NULL);
  /*
*return the number of characters that aren't delims
*/
  n = strcspn(saveline, delims);
  p = saveline; /*save start of this token*/

  saveline += n; /*bump past the delim*/

  if(*saveline != '\0') /*trash the delim if necessary*/
    *saveline++ = '\0';

  return(p);
}

} // namespace ar

#endif //AR13_TSVIO_H
