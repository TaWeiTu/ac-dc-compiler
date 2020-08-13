#include <algorithm>
#include <cassert>
#include <climits>
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
  DECL_INT,
  DECL_FLOAT,
  CMD_PRINT,
  CONST_INT,
  CONST_FLOAT,
  PAREN_L,
  PAREN_R,
  IDENTIFIER,
  BIN_OP_ADD,
  BIN_OP_SUB,
  BIN_OP_MUL,
  BIN_OP_DIV,
  BIN_OP_ASSIGN,
  END_OF_FILE
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

  Token getNextToken();
  Token getNumericToken(char C);
  Token getDeclOrIDToken(char C);
  Token getParenOrOpToken(char C);

public:
  Tokenizer(std::ifstream &&S) : IFS(std::move(S)), CurToken(getNextToken()) {}

  operator bool() const { return CurToken.Type != END_OF_FILE; }
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
  std::string V;
  while (isalpha(C)) {
    V += C;
    C = IFS.get();
  }
  Token Tok;
  if (V.size() == 1 && (V[0] == 'i' || V[0] == 'f' || V[0] == 'p')) {
    Tok.Type = V[0] == 'i' ? DECL_INT : V[0] == 'f' ? DECL_FLOAT : CMD_PRINT;
  } else {
    Tok.Type = IDENTIFIER;
    Tok.Value = std::move(V);
  }
  IFS.unget();
  return Tok;
}

Token Tokenizer::getParenOrOpToken(char C) {
  switch (C) {
  case '+':
    return Token{BIN_OP_ADD};
  case '-':
    return Token{BIN_OP_SUB};
  case '*':
    return Token{BIN_OP_MUL};
  case '/':
    return Token{BIN_OP_DIV};
  case '=':
    return Token{BIN_OP_ASSIGN};
  case '(':
    return Token{PAREN_L};
  case ')':
    return Token{PAREN_R};
  default:
    emitError("Tokenizer", "expecting '+', '-', '*', '/' or '=', but '", C,
              "' found.");
  }
}

Token Tokenizer::getNextToken() {
  char C = IFS.get();
  while (isspace(C))
    C = IFS.get();

  if (C == EOF)
    return Token{END_OF_FILE};
  if (isdigit(C))
    return getNumericToken(C);
  if (isalpha(C))
    return getDeclOrIDToken(C);

  return getParenOrOpToken(C);
}

const Token &Tokenizer::peekToken() { return CurToken; }

Token Tokenizer::readToken() {
  Token Result(std::move(CurToken));
  CurToken = getNextToken();
  return Result;
}

std::ostream &operator<<(std::ostream &S, TokenType T) {
  if (T == DECL_INT)
    return S << "DECL_INT";
  if (T == DECL_FLOAT)
    return S << "DECL_FLOAT";
  if (T == CMD_PRINT)
    return S << "CMD_PRINT";
  return S;
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
  BIN_MUL_NODE,
  BIN_DIV_NODE,
  CONVERSION_NODE
};

enum DataType { DATA_INT, DATA_FLOAT };

struct AST {
  ASTNodeType Type;
  std::vector<AST *> SubTree;
  std::variant<int, float, size_t, DataType> Value;

  AST(ASTNodeType T) : Type(T) {}
  AST(ASTNodeType T, AST *Child) : Type(T), SubTree({Child}) {}

  ~AST() {
    for (AST *Node : SubTree)
      delete Node;
    SubTree.clear();
  }
};

class SymbolTable {
  std::unordered_map<std::string, size_t> VarID;
  std::vector<VariableType> VarType;

public:
  size_t declareSymbol(const std::string &ID, TokenType DeclType) {
    if (VarID.find(ID) != VarID.end())
      emitError("SymbolTable", "redeclaration of variable ", ID);
    size_t Res = (VarID[ID] = VarType.size());
    VarType.push_back((DeclType == DECL_INT ? VAR_INT : VAR_FLOAT));
    return Res;
  }

  size_t getVarID(const std::string &ID) const {
    auto Ptr = VarID.find(ID);
    if (Ptr == VarID.end())
      emitError("SymbolTable", "use of undeclared variable ", ID);
    return Ptr->second;
  }

  VariableType getVarType(const std::string &ID) const {
    return VarType[getVarID(ID)];
  }
  VariableType getVarType(size_t V) const { return VarType[V]; }
};

class Parser {
  Tokenizer TK;
  SymbolTable ST;
  void reduceStack(std::vector<AST *> &Stack);
  AST *parseStatement();
  AST *parseAssignment(const std::string &ID);
  AST *parseDeclaration(TokenType DeclType);
  AST *parseExpression();

  DataType getDataType(AST *Node) const;
  DataType promoteType(AST *&LHS, AST *&RHS) const;

public:
  Parser(std::ifstream &&S) : TK(std::move(S)) {}
  const SymbolTable &getSymbolTable() const { return ST; }

  AST *parse();
};

AST *Parser::parse() {
  AST *Node = new AST(PROGRAM_NODE);
  while (TK)
    Node->SubTree.push_back(parseStatement());
  return Node;
}

AST *Parser::parseStatement() {
  Token Tok = TK.readToken();
  if (Tok.Type == DECL_INT || Tok.Type == DECL_FLOAT)
    return parseDeclaration(Tok.Type);
  if (Tok.Type == IDENTIFIER)
    return parseAssignment(Tok.Value);

  if (Tok.Type != CMD_PRINT)
    emitError("Parser",
              "expecting DECL_INT, DECL_FLOAT, IDENTIFIER or CMD_PRINT, but ",
              Tok.Type, " found.");
  Tok = TK.readToken();
  if (Tok.Type != IDENTIFIER)
    emitError("Parser", "expecting IDENTIFIER, but ", Tok.Type, " found.");

  size_t Var = ST.getVarID(Tok.Value);
  AST *Node = new AST(PRINTSTMT_NODE);
  Node->Value = Var;
  return Node;
}

AST *Parser::parseDeclaration(TokenType DeclType) {
  Token Tok = TK.readToken();
  if (Tok.Type != IDENTIFIER)
    emitError("Parser", "expecting IDENTIFIER, but ", DeclType, " found.");
  AST *Node = new AST(DECLARATION_NODE);
  Node->Value = ST.declareSymbol(Tok.Value, DeclType);
  return Node;
}

AST *Parser::parseAssignment(const std::string &ID) {
  size_t Var = ST.getVarID(ID);
  if (auto T = TK.readToken().Type; T != BIN_OP_ASSIGN)
    emitError("Parser", "expecting BIN_OP_ASSIGN, but ", T, " found.");

  AST *Expr = parseExpression();
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
  if (Node->Type == DECLARATION_NODE)
    return ST.getVarType(std::get<size_t>(Node->Value)) == VAR_INT ? DATA_INT
                                                                   : DATA_FLOAT;
  if (Node->Type == BIN_ADD_NODE || Node->Type == BIN_SUB_NODE ||
      Node->Type == BIN_MUL_NODE || Node->Type == BIN_DIV_NODE)
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

void Parser::reduceStack(std::vector<AST *> &Stack) {
  std::vector<AST *> Nodes;
  while (Stack.size() >= 2 && (Stack[Stack.size() - 2]->Type == BIN_MUL_NODE ||
                               Stack[Stack.size() - 2]->Type == BIN_DIV_NODE)) {
    Nodes.push_back(Stack.back());
    Stack.pop_back();
    Nodes.push_back(Stack.back());
    Stack.pop_back();
  }
  assert(!Stack.empty());
  Nodes.push_back(Stack.back());
  Stack.pop_back();
  std::reverse(Nodes.begin(), Nodes.end());
  AST *Prev = Nodes[0];
  for (size_t i = 1; i < Nodes.size(); i += 2) {
    assert(Nodes[i]->Type == BIN_MUL_NODE || Nodes[i]->Type == BIN_DIV_NODE);
    AST *LHS = Prev, *RHS = Nodes[i + 1];
    Nodes[i]->Value = promoteType(LHS, RHS);
    Nodes[i]->SubTree = {LHS, RHS};
    Prev = Nodes[i];
  }
  Stack.push_back(Prev);
}

AST *Parser::parseExpression() {
  std::vector<AST *> Stack;
  bool IsValue = true, Terminated = false;
  while (!Terminated) {
    TokenType Type = TK.peekToken().Type;
    if (IsValue) {
      switch (Type) {
      case PAREN_L: {
        TK.readToken();
        Stack.push_back(parseExpression());
        if (auto T = TK.readToken().Type; T != PAREN_R)
          emitError("Parser", "expecting PAREN_R, but ", T, " found.");
        break;
      }
      case IDENTIFIER: {
        std::string ID = TK.readToken().Value;
        AST *Node = new AST(IDENTIFIER_NODE);
        Node->Value = ST.getVarID(ID);
        Stack.push_back(Node);
        break;
      }
      case CONST_INT: {
        std::string Value = TK.readToken().Value;
        AST *Node = new AST(CONST_INT_NODE);
        Node->Value = std::stoi(Value);
        Stack.push_back(Node);
        break;
      }
      case CONST_FLOAT: {
        std::string Value = TK.readToken().Value;
        AST *Node = new AST(CONST_FLOAT_NODE);
        Node->Value = std::stof(Value);
        Stack.push_back(Node);
        break;
      }
      default:
        emitError("Parser", "unexpected token type.");
      }
    } else {
      switch (Type) {
      case BIN_OP_ADD:
      case BIN_OP_SUB:
        TK.readToken();
        reduceStack(Stack);
        Stack.push_back(
            new AST(Type == BIN_OP_ADD ? BIN_ADD_NODE : BIN_SUB_NODE));
        break;
      case BIN_OP_MUL:
      case BIN_OP_DIV:
        TK.readToken();
        Stack.push_back(
            new AST(Type == BIN_OP_MUL ? BIN_MUL_NODE : BIN_DIV_NODE));
        break;
      default:
        Terminated = true;
        break;
      }
    }
    IsValue = !IsValue;
  }
  reduceStack(Stack);
  assert(Stack.size() % 2 == 1);
  AST *Prev = Stack[0];
  for (size_t i = 1; i < Stack.size(); i += 2) {
    assert(Stack[i]->Type == BIN_ADD_NODE || Stack[i]->Type == BIN_SUB_NODE);
    Stack[i]->SubTree = {Prev, Stack[i + 1]};
    Stack[i]->Value = promoteType(Prev, Stack[i + 1]);
    Prev = Stack[i];
  }
  return Prev;
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

char printOperator(ASTNodeType Type) {
  if (Type == BIN_ADD_NODE)
    return '+';
  if (Type == BIN_SUB_NODE)
    return '-';
  if (Type == BIN_MUL_NODE)
    return '*';

  return '/';
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
  case BIN_SUB_NODE:
  case BIN_MUL_NODE:
  case BIN_DIV_NODE: {
    assert(Expr->SubTree.size() == 2);
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

template <typename T> T fold(T LHS, T RHS, ASTNodeType Op) {
  if (Op == BIN_ADD_NODE)
    return LHS + RHS;
  if (Op == BIN_SUB_NODE)
    return LHS - RHS;
  if (Op == BIN_MUL_NODE)
    return LHS * RHS;
  if (Op == BIN_DIV_NODE)
    return LHS / RHS;

  emitError("Optimizer", "unexpected binary operation.");
}

void optExpression(AST *Expr) {
  if (Expr->Type == CONVERSION_NODE) {
    AST *Child = Expr->SubTree[0];
    optExpression(Child);
    assert(Child->Type != CONST_FLOAT_NODE);
    if (Child->Type == CONST_INT_NODE) {
      Expr->Type = CONST_FLOAT_NODE;
      Expr->Value = static_cast<float>(std::get<int>(Child->Value));
      Expr->SubTree.clear();
      delete Child;
    }
    return;
  }

  if (Expr->SubTree.size() != 2)
    return;

  assert(Expr->Type == BIN_ADD_NODE || Expr->Type == BIN_SUB_NODE ||
         Expr->Type == BIN_MUL_NODE || Expr->Type == BIN_DIV_NODE);
  AST *LHS = Expr->SubTree[0], *RHS = Expr->SubTree[1];
  optExpression(LHS);
  optExpression(RHS);

  bool ConstIntLHS = LHS->Type == CONST_INT_NODE;
  bool ConstIntRHS = RHS->Type == CONST_INT_NODE;
  if (!ConstIntLHS || !ConstIntRHS)
    return;

  int64_t LV = std::get<int>(LHS->Value), RV = std::get<int>(RHS->Value);
  int64_t Result = fold(LV, RV, Expr->Type);
  if (Result >= INT_MIN && Result <= INT_MAX) {
    Expr->Type = CONST_INT_NODE;
    Expr->Value = static_cast<int>(Result);
    Expr->SubTree.clear();
    delete LHS;
    delete RHS;
  }
}

void optProgram(AST *Program) {
  for (AST *Node : Program->SubTree) {
    if (Node->Type == ASSIGNMENT_NODE) {
      AST *Expr = Node->SubTree[0];
      optExpression(Expr);
    }
  }
}

int main(int argc, const char **argv) {
  if (argc != 3 && argc != 4) {
    std::cerr << "[Usage] " << argv[0] << " source_file target_file [-O]\n";
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
  bool Opt = false;
  if (argc == 4) {
    if (strcmp(argv[3], "-O") != 0) {
      std::cerr << "[Error] expecting -O, but " << argv[3] << " found.\n";
      exit(1);
    }
    Opt = true;
  }
  Parser PS(std::move(Source));
  AST *Program = PS.parse();

  if (Opt)
    optProgram(Program);

  DCCodeGen DCG(PS.getSymbolTable(), std::move(Target));
  DCG.genProgram(Program);
  delete Program;
  return 0;
}
