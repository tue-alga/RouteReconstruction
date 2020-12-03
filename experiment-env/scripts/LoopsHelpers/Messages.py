# Message types
class Messages:
    MSG_RESUME = 'resume'
    MSG_NEWBASIS = 'newbasis'
    MSG_BASISSIZE = 'basissize'
    MSG_NNLSAPPLIED = 'nnlsapplied'
    MSG_INITNORM = 'initnorm'

    RESULT_FILE_RE = 'it=([0-9]+)\-([0-9]+)_(newbasis|nnlsApplied)(?:\-)?([a-zA-Z]+)*\.boosttxt'

    NNLS_FILENAME_FORMAT = 'it={}-{}_nnlsApplied.boosttxt' # Args: iteration and phase
    ALG_RESULT_FILENAME_FORMAT = 'it={}-{}_newbasis-{}.boosttxt' # Args: iteration, phase and alg name