-- Zachary M. Mattis
-- ER Modeling Relational Databases

-- Customer Entity
CREATE TABLE TBL_CUSTOMER (
    SSN INTEGER NOT NULL,
    FIRST_NAME CHAR(30) NOT NULL,
    MIDDLE_INITIAL CHAR(1),
    LAST_NAME CHAR(30) NOT NULL,
    POLICY_ID INTEGER NOT NULL,
    CONSTRAINT PK_CUSTOMER PRIMARY KEY (SSN),
    CONSTRAINT FK_CUSTOMER_POLICY FOREIGN KEY (POLICY_ID) REFERENCES TBL_POLICY(POLICY_ID)
);

-- Policy Entity
CREATE TABLE TBL_POLICY (
    POLICY_ID INTEGER NOT NULL,
    DEDUCTIBLE DECIMAL(9,0) NOT NULL,
    STARTING_DATE DATE NOT NULL,
    EXPIRATION_DATE DATE NOT NULL,
    CONSTRAINT PK_POLICY PRIMARY KEY (POLICY_ID)
);

-- Car Entity
CREATE TABLE TBL_CAR (
    VIN CHAR(17) NOT NULL,
    MAKE CHAR(30) NOT NULL,
    MODEL CHAR(30) NOT NULL,
    YEAR INTEGER NOT NULL,
    COLOR CHAR(30) NOT NULL,
    POLICY_ID INTEGER NOT NULL,
    CONSTRAINT PK_CAR PRIMARY KEY (VIN),
    CONSTRAINT FK_CAR_POLICY FOREIGN KEY (POLICY_ID) REFERENCES TBL_POLICY(POLICY_ID)
);

-- Claim Entity
CREATE TABLE TBL_CLAIM (
    CLAIM_ID INTEGER NOT NULL,
    CLAIM_DATE DATE NOT NULL,
    DAMAGE_AMOUNT DECIMAL(9,0) NOT NULL,
    CUSTOMER_ID INTEGER NOT NULL,
    CONSTRAINT PK_CLAIM PRIMARY KEY (CLAIM_ID),
    CONSTRAINT FK_CLAIM_CUSTOMER FOREIGN KEY (CUSTOMER_ID) REFERENCES TBL_CUSTOMER(SSN)
);