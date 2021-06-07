#include <exception>
#include <string>

class DingoException : virtual public std::exception
{

protected:
    int number;
    int offset;
    std::string message;

public:
    /** Constructor.
     *  @param msg The error message
     *  @param err_num Error number
     *  @param err_off Error offset
     */
    explicit DingoException(const std::string &msg, int err_num, int err_off) : number(err_num), offset(err_off), message(msg)
    {
    }

    /** Destructor. */
    virtual ~DingoException() throw() {}

    /** Returns error message.
     *  @return #message
     */
    virtual const char *what() const throw()
    {
        return message.c_str();
    }

    /** Returns error number.
     *  @return #number
     */
    virtual int getErrorNumber() const throw()
    {
        return number;
    }

    /** Returns error offset.
     *  @return #offset
     */
    virtual int getErrorOffset() const throw()
    {
        return offset;
    }
};