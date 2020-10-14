#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

enum TokenType {
  DECL_INT,    // integer declaration ("i")
  DECL_FLOAT,  // float declaration ("f")
  CMD_PRINT,   // print command ("p")
  CONST_INT,   // integer constant
  CONST_FLOAT, // float constant
  IDENTIFIER,
  BIN_OP_ADD,    // addition operator ("+")
  BIN_OP_SUB,    // subtraction operator ("-")
  BIN_OP_ASSIGN, // assignment operator ("=")
  END_OF_FILE    // EOF
};

struct Token {
  TokenType Type;
  std::string Value;
};

template <typename... ArgsT> void emitErrorImpl(ArgsT &&... Args);
template <> void emitErrorImpl<>() {}

template <typename HeadT, typename... TailT>
void emitErrorImpl(HeadT &&H, TailT &&... T) {
  std::cerr << H;
  emitErrorImpl(std::forward<TailT>(T)...);
}

/// Helper function of emitting the error to standard error.
template <typename... ArgsT>
[[noreturn]] void emitError(std::string E, ArgsT &&... Args) {
  std::cerr << "[Error] " << E << ": ";
  emitErrorImpl(std::forward<ArgsT>(Args)...);
  std::cerr << "\n";
  exit(1);
}

class Tokenizer {
  std::ifstream IFS;
  Token CurToken;
  size_t NumWhiteSpaces;

  Token getNextToken();
  Token getNumericToken(char C);
  Token getDeclOrIDToken(char C);
  Token getOpToken(char C) const;

public:
  Tokenizer(std::ifstream &&S)
      : IFS(std::move(S)), CurToken(getNextToken()), NumWhiteSpaces(0) {}

  operator bool() const { return CurToken.Type != END_OF_FILE; }
  bool hasReadWhiteSpaces() const { return NumWhiteSpaces != 0; }
  Token readToken();
  const Token &peekToken();
};

Token Tokenizer::getNumericToken(char C) {
  Token Tok;
  while (isdigit(C)) {
    Tok.Value += C;
    C = IFS.get();
  }
  if (C != '.') {
    Tok.Type = CONST_INT;
  } else {
    Tok.Type = CONST_FLOAT;
    Tok.Value += '.';
    while (C = IFS.get(), isdigit(C))
      Tok.Value += C;
  }
  IFS.unget();
  return Tok;
}

Token Tokenizer::getDeclOrIDToken(char C) {
  std::string Value = "";
  while (isalpha(C)) {
    Value += C;
    C = IFS.get();
  }
  IFS.unget();
  if (Value.size() > 1)
    emitError("Tokenizer", "expecting single-character token.");
  Token Tok;
  switch (Value[0]) {
  case 'i':
    Tok.Type = DECL_INT;
    break;
  case 'f':
    Tok.Type = DECL_FLOAT;
    break;
  case 'p':
    Tok.Type = CMD_PRINT;
    break;
  default:
    Tok.Type = IDENTIFIER;
    Tok.Value = Value;
    break;
  }
  return Tok;
}

Token Tokenizer::getOpToken(char C) const {
  switch (C) {
  case '+':
    return Token{BIN_OP_ADD};
  case '-':
    return Token{BIN_OP_SUB};
  case '=':
    return Token{BIN_OP_ASSIGN};
  default:
    emitError("Tokenizer", "expecting '+', '-', or '=', but '", C, "' found.");
  }
}

Token Tokenizer::getNextToken() {
  char C = IFS.get();
  NumWhiteSpaces = 0;
  while (isspace(C)) {
    C = IFS.get();
    NumWhiteSpaces++;
  }

  if (C == EOF)
    return Token{END_OF_FILE};
  if (isdigit(C))
    return getNumericToken(C);
  if (isalpha(C))
    return getDeclOrIDToken(C);

  return getOpToken(C);
}

const Token &Tokenizer::peekToken() { return CurToken; }

std::ostream &operator<<(std::ostream &S, TokenType T) {
  switch (T) {
  case DECL_INT:
    return S << "DECL_INT";
  case DECL_FLOAT:
    return S << "DECL_FLOAT";
  case CMD_PRINT:
    return S << "CMD_PRINT";
  case CONST_INT:
    return S << "CONST_INT";
  case CONST_FLOAT:
    return S << "CONST_FLOAT";
  case IDENTIFIER:
    return S << "IDENTIFIER";
  case BIN_OP_ADD:
    return S << "BIN_OP_ADD";
  case BIN_OP_SUB:
    return S << "BIN_OP_SUB";
  case BIN_OP_ASSIGN:
    return S << "BIN_OP_ASSIGN";
  case END_OF_FILE:
    return S << "END_OF_FILE";
  }
  __builtin_unreachable();
}

Token Tokenizer::readToken() {
  Token Result(std::move(CurToken));
  CurToken = getNextToken();
  return Result;
}

enum VariableType { VAR_INT, VAR_FLOAT };

enum ASTNodeType {
  PROGRAM_NODE,
  DECLARATION_NODE,
  ASSIGNMENT_NODE,
  PRINTSTMT_NODE,
  IDENTIFIER_NODE,
  CONST_INT_NODE,
  CONST_FLOAT_NODE,
  BIN_ADD_NODE,
  BIN_SUB_NODE,
  CONVERSION_NODE
};

enum DataType { DATA_INT, DATA_FLOAT };

struct AST {
  ASTNodeType Type;
  std::vector<AST *> SubTree;
  std::variant<int, float, size_t, DataType> Value;

  AST() = default;
  AST(ASTNodeType T) : Type(T) {}
  AST(ASTNodeType T, AST *Child) : Type(T), SubTree({Child}) {}

  ~AST() {
    for (AST *Node : SubTree)
      delete Node;
    SubTree.clear();
  }
};

class SymbolTable {
  std::unordered_map<char, size_t> VarID;
  std::vector<VariableType> VarType;

public:
  size_t declareSymbol(char ID, TokenType DeclType) {
    if (VarID.find(ID) != VarID.end())
      emitError("SymbolTable", "redeclaration of variable ", ID);
    size_t Res = (VarID[ID] = VarType.size());
    VarType.push_back((DeclType == DECL_INT ? VAR_INT : VAR_FLOAT));
    return Res;
  }

  size_t getVarID(char ID) const {
    auto Iter = VarID.find(ID);
    if (Iter == VarID.end())
      emitError("SymbolTable", "use of undeclared identifier ", ID);
    return Iter->second;
  }

  VariableType getVarType(char ID) const { return VarType[getVarID(ID)]; }
  VariableType getVarType(size_t V) const { return VarType[V]; }
};

class Parser {
  Tokenizer TK;
  SymbolTable ST;
  AST *parseStatement();
  AST *parseAssignment(char ID);
  AST *parseDeclaration(TokenType DeclType);
  AST *parseValue();
  AST *parseExpression(AST *LHS);

  DataType getDataType(AST *Node) const;
  DataType promoteType(AST *&LHS, AST *&RHS) const;

public:
  Parser(std::ifstream &&S) : TK(std::move(S)) {}
  const SymbolTable &getSymbolTable() const { return ST; }

  AST *parse();
};

AST *Parser::parse() {
  AST *Node = new AST(PROGRAM_NODE);
  bool EndOfDecl = false, FirstStmt = true;
  while (TK) {
    AST *Stmt = parseStatement();
    if (!FirstStmt && !TK.hasReadWhiteSpaces())
      emitError("Parser", "expecting white spaces at the end of a statement.");
    FirstStmt = false;
    if (EndOfDecl && Stmt->Type == DECLARATION_NODE)
      emitError("Parser",
                "declarations should come before other types of statements.");
    if (Stmt->Type != DECLARATION_NODE)
      EndOfDecl = true;
    Node->SubTree.push_back(Stmt);
  }
  return Node;
}

AST *Parser::parseStatement() {
  Token Tok = TK.readToken();
  if (Tok.Type == DECL_INT || Tok.Type == DECL_FLOAT)
    return parseDeclaration(Tok.Type);

  if (Tok.Type == IDENTIFIER) {
    assert(Tok.Value.size() == 1 &&
           "Variable names with multiple characters are not supported.");
    return parseAssignment(Tok.Value[0]);
  }

  if (Tok.Type != CMD_PRINT)
    emitError("Parser",
              "expecting DECL_INT, DECL_FLOAT, IDENTIFIER or CMD_PRINT, but ",
              Tok.Type, " found.");
  Tok = TK.readToken();
  if (Tok.Type != IDENTIFIER)
    emitError("Parser", "expecting IDENTIFIER, but ", Tok.Type, " found.");

  // Print statement.
  assert(Tok.Value.size() == 1 &&
         "Variable names with multiple characters are not supported.");
  size_t Var = ST.getVarID(Tok.Value[0]);
  AST *Node = new AST(PRINTSTMT_NODE);
  Node->Value = Var;
  return Node;
}

AST *Parser::parseDeclaration(TokenType DeclType) {
  Token Tok = TK.readToken();
  if (Tok.Type != IDENTIFIER)
    emitError("Parser", "expecting IDENTIFIER, but ", DeclType, " found.");
  AST *Node = new AST(DECLARATION_NODE);
  assert(Tok.Value.size() == 1 &&
         "Variable names with multiple characters are not supported.");
  Node->Value = ST.declareSymbol(Tok.Value[0], DeclType);
  return Node;
}

AST *Parser::parseAssignment(char ID) {
  size_t Var = ST.getVarID(ID);
  if (TokenType T = TK.readToken().Type; T != BIN_OP_ASSIGN)
    emitError("Parser", "expecting BIN_OP_ASSIGN, but ", T, " found.");

  AST *LHS = parseValue();
  AST *Expr = parseExpression(LHS);
  if (getDataType(Expr) == DATA_FLOAT && ST.getVarType(Var) == VAR_INT)
    emitError("Parser", "cannot convert float to integer.");
  AST *Node = new AST(ASSIGNMENT_NODE, Expr);
  Node->Value = Var;
  return Node;
}

DataType Parser::getDataType(AST *Node) const {
  if (Node->Type == CONST_INT_NODE)
    return DATA_INT;
  if (Node->Type == CONST_FLOAT_NODE)
    return DATA_FLOAT;
  if (Node->Type == IDENTIFIER_NODE)
    return ST.getVarType(std::get<size_t>(Node->Value)) == VAR_INT ? DATA_INT
                                                                   : DATA_FLOAT;
  if (Node->Type == BIN_ADD_NODE || Node->Type == BIN_SUB_NODE)
    return std::get<DataType>(Node->Value);

  emitError("Parser", "unexpected AST node type.");
}

DataType Parser::promoteType(AST *&LHS, AST *&RHS) const {
  DataType L = getDataType(LHS);
  DataType R = getDataType(RHS);
  if (L == R)
    return L;

  AST *Cvt = new AST(CONVERSION_NODE);
  Cvt->Value = DATA_FLOAT;
  if (L == DATA_INT) {
    Cvt->SubTree = {LHS};
    LHS = Cvt;
  } else {
    Cvt->SubTree = {RHS};
    RHS = Cvt;
  }
  return DATA_FLOAT;
}

AST *Parser::parseValue() {
  Token Tok = TK.readToken();
  AST *Node = new AST();
  switch (Tok.Type) {
  case CONST_INT:
    Node->Type = CONST_INT_NODE;
    Node->Value = std::stoi(Tok.Value);
    break;
  case CONST_FLOAT:
    Node->Type = CONST_FLOAT_NODE;
    Node->Value = std::stof(Tok.Value);
    break;
  case IDENTIFIER:
    Node->Type = IDENTIFIER_NODE;
    assert(Tok.Value.size() == 1 &&
           "Variable names with multiple characters are not supported.");
    Node->Value = ST.getVarID(Tok.Value[0]);
    break;
  default:
    emitError("Parser", "expecting CONST_INT, CONST_FLOAT or IDENTIFIER, but ",
              Tok.Type, " found.");
  }
  return Node;
}

AST *Parser::parseExpression(AST *LHS) {
  TokenType Type = TK.peekToken().Type;
  if (Type != BIN_OP_ADD && Type != BIN_OP_SUB)
    return LHS;

  TK.readToken();
  AST *Node = new AST(Type == BIN_OP_ADD ? BIN_ADD_NODE : BIN_SUB_NODE);
  AST *RHS = parseValue();
  Node->Value = promoteType(LHS, RHS);
  Node->SubTree = {LHS, RHS};
  return parseExpression(Node);
}

class DCCodeGen {
  const SymbolTable &ST;
  std::ofstream OFS;
  enum Precision { PREC_INT, PREC_FLOAT, PREC_UNKNOWN };
  Precision CurPrec = PREC_UNKNOWN;

  void genStatement(AST *Stmt);
  void genAssignment(AST *Stmt);
  void genExpression(AST *Expr);
  void genPrintStmt(AST *Stmt);

public:
  DCCodeGen(const SymbolTable &ST, std::ofstream &&S)
      : ST(ST), OFS(std::move(S)) {
    OFS << std::fixed << std::setprecision(5);
  }

  void genProgram(AST *Program);
};

void DCCodeGen::genProgram(AST *Program) {
  assert(Program->Type == PROGRAM_NODE);
  for (AST *Node : Program->SubTree)
    genStatement(Node);
}

void DCCodeGen::genStatement(AST *Stmt) {
  if (Stmt->Type == ASSIGNMENT_NODE)
    return genAssignment(Stmt);
  if (Stmt->Type == PRINTSTMT_NODE)
    return genPrintStmt(Stmt);
}

void DCCodeGen::genAssignment(AST *Stmt) {
  assert(Stmt->SubTree.size() == 1);
  AST *Expr = Stmt->SubTree[0];
  genExpression(Expr);
  OFS << "s" << static_cast<char>('a' + std::get<size_t>(Stmt->Value)) << "\n";
}

inline char printOperator(ASTNodeType Type) {
  return Type == BIN_ADD_NODE ? '+' : '-';
}

void DCCodeGen::genExpression(AST *Expr) {
  switch (Expr->Type) {
  case CONVERSION_NODE:
    return genExpression(Expr->SubTree[0]);
  case IDENTIFIER_NODE:
    OFS << "l" << static_cast<char>('a' + std::get<size_t>(Expr->Value))
        << "\n";
    return;
  case CONST_INT_NODE:
    OFS << std::get<int>(Expr->Value) << "\n";
    return;
  case CONST_FLOAT_NODE:
    OFS << std::get<float>(Expr->Value) << "\n";
    return;
  case BIN_ADD_NODE:
  case BIN_SUB_NODE: {
    assert(Expr->SubTree.size() == 2 &&
           "Expecting two children for binary expressions.");
    genExpression(Expr->SubTree[0]);
    genExpression(Expr->SubTree[1]);
    Precision Prec =
        std::get<DataType>(Expr->Value) == DATA_INT ? PREC_INT : PREC_FLOAT;
    if (Prec != CurPrec) {
      OFS << (Prec == PREC_INT ? 0 : 5) << " k\n";
      CurPrec = Prec;
    }
    OFS << printOperator(Expr->Type) << "\n";
    return;
  }
  default:
    emitError("CodeGen", "unexpected AST node type.");
  }
}

void DCCodeGen::genPrintStmt(AST *Stmt) {
  assert(Stmt->SubTree.empty());
  OFS << "l" << static_cast<char>('a' + std::get<size_t>(Stmt->Value)) << "\n";
  OFS << "p\n";
}

int main(int argc, const char **argv) {
  if (argc != 3) {
    std::cerr << "[Usage] " << argv[0] << " source_file target_file\n";
    exit(1);
  }
  std::ifstream Source(argv[1]);
  if (!Source) {
    std::cerr << "[Error] failed to open source_file\n";
    exit(1);
  }
  std::ofstream Target(argv[2]);
  if (!Target) {
    std::cerr << "[Error] failed to open target_file\n";
    exit(1);
  }
  Parser PS(std::move(Source));
  AST *Program = PS.parse();
  DCCodeGen DCG(PS.getSymbolTable(), std::move(Target));
  DCG.genProgram(Program);
  delete Program;
  return 0;
}
